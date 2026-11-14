#include "audio2_stream/WasapiProxyImpl.hpp"
#include "audio2_stream/timestamp.hpp"

#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <mmreg.h>
#include <ksmedia.h>
#include <wrl/client.h>
// Defines PKEY_Device_FriendlyName in this translation unit instead of requiring a GUID library
#include <initguid.h>
#include <functiondiscoverykeys_devpkey.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cwctype>
#include <format>

using Microsoft::WRL::ComPtr;

std::unique_ptr<IAudioDeviceProxy> make_default_device_proxy()
{
  return std::make_unique<WasapiProxyImpl>();
}

namespace
{

// Wait on the buffer event this long before re-checking the shutdown flag
const DWORD EVENT_WAIT_MS = 100;
// Extra time allowed for playback to drain beyond the buffer duration
const int DRAIN_MARGIN_MS = 200;

// COM must be initialized on each thread that uses it. open(), write() and read() may run on
// different threads (node callbacks vs stream threads). COM is intentionally never uninitialized:
// the objects can be released from any thread, which requires the multithreaded apartment to
// stay alive.
void ensure_com()
{
  thread_local bool initialized = false;
  if (!initialized) {
    // The result is ignored: RPC_E_CHANGED_MODE means the thread is already a
    // single-threaded apartment, which WASAPI also supports. Any other failure
    // surfaces as CO_E_NOTINITIALIZED from the next WASAPI call.
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    initialized = true;
  }
}

std::wstring to_wide(const std::string & str)
{
  if (str.empty()) {
    return std::wstring();
  }
  int len = MultiByteToWideChar(CP_UTF8, 0, str.data(), static_cast<int>(str.size()), nullptr, 0);
  std::wstring wide(len, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, str.data(), static_cast<int>(str.size()), wide.data(), len);
  return wide;
}

std::string to_utf8(const std::wstring & wide)
{
  if (wide.empty()) {
    return std::string();
  }
  int len = WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), nullptr, 0,
    nullptr, nullptr);
  std::string str(len, '\0');
  WideCharToMultiByte(CP_UTF8, 0, wide.data(), static_cast<int>(wide.size()), str.data(), len,
    nullptr, nullptr);
  return str;
}

std::wstring to_lower(std::wstring str)
{
  std::transform(str.begin(), str.end(), str.begin(),
    [](wchar_t c) {return static_cast<wchar_t>(std::towlower(c));});
  return str;
}

std::string hresult_message(HRESULT hr)
{
  // WASAPI-specific codes have no system message text
  switch (hr) {
    case AUDCLNT_E_DEVICE_INVALIDATED: return "audio device was removed or disabled";
    case AUDCLNT_E_UNSUPPORTED_FORMAT: return "unsupported audio format";
    case AUDCLNT_E_DEVICE_IN_USE: return "audio device is in use";
    case AUDCLNT_E_NOT_INITIALIZED: return "audio client is not initialized";
    case AUDCLNT_E_ALREADY_INITIALIZED: return "audio client is already initialized";
    case AUDCLNT_E_SERVICE_NOT_RUNNING: return "Windows Audio service is not running";
    case AUDCLNT_E_ENDPOINT_CREATE_FAILED: return "cannot create audio endpoint";
    case AUDCLNT_E_BUFFER_SIZE_ERROR: return "invalid buffer size";
    case AUDCLNT_E_BUFFER_TOO_LARGE: return "buffer request too large";
    case AUDCLNT_E_WRONG_ENDPOINT_TYPE: return "wrong endpoint type";
    default: break;
  }
  char * buffer = nullptr;
  DWORD len = FormatMessageA(
    FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
    nullptr, static_cast<DWORD>(hr), 0, reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
  std::string message = len > 0 ? std::string(buffer, len) : std::string("unknown error");
  LocalFree(buffer);
  while (!message.empty() && (message.back() == '\n' || message.back() == '\r' ||
    message.back() == ' ' || message.back() == '.'))
  {
    message.pop_back();
  }
  return message;
}

std::string hresult_error(const std::string & context, HRESULT hr)
{
  return std::format("WASAPI {}: {} (0x{:08X})", context, hresult_message(hr),
           static_cast<unsigned long>(hr));
}

EDataFlow to_data_flow(AudioDirection direction)
{
  return direction == AudioDirection::Capture ? eCapture : eRender;
}

const char * direction_name(AudioDirection direction)
{
  return direction == AudioDirection::Capture ? "capture" : "playback";
}

bool make_wave_format(
  SampleFormat format, unsigned int channels, unsigned int samplerate,
  WAVEFORMATEXTENSIBLE & wfx)
{
  auto sample_format = wasapi_sample_format(format);
  if (!sample_format) {
    return false;
  }
  wfx = {};
  wfx.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  wfx.Format.nChannels = static_cast<WORD>(channels);
  wfx.Format.nSamplesPerSec = samplerate;
  wfx.Format.wBitsPerSample = sample_format->container_bits;
  wfx.Format.nBlockAlign = static_cast<WORD>(channels * sample_format->container_bits / 8);
  wfx.Format.nAvgBytesPerSec = samplerate * wfx.Format.nBlockAlign;
  wfx.Format.cbSize = sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX);
  wfx.Samples.wValidBitsPerSample = sample_format->valid_bits;
  if (channels == 1) {
    wfx.dwChannelMask = SPEAKER_FRONT_CENTER;
  } else if (channels == 2) {
    wfx.dwChannelMask = SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
  } else {
    wfx.dwChannelMask = 0;
  }
  wfx.SubFormat = sample_format->is_float ? KSDATAFORMAT_SUBTYPE_IEEE_FLOAT : KSDATAFORMAT_SUBTYPE_PCM;
  return true;
}

// Visit each active endpoint for a direction with its ID and friendly name.
// The visitor returns true to stop.
template<typename Visitor>
HRESULT for_each_endpoint(IMMDeviceEnumerator * enumerator, EDataFlow flow, Visitor visit)
{
  ComPtr<IMMDeviceCollection> collection;
  HRESULT hr = enumerator->EnumAudioEndpoints(flow, DEVICE_STATE_ACTIVE, &collection);
  if (FAILED(hr)) {
    return hr;
  }
  UINT count = 0;
  collection->GetCount(&count);
  for (UINT i = 0; i < count; i++) {
    ComPtr<IMMDevice> device;
    if (FAILED(collection->Item(i, &device))) {
      continue;
    }
    std::wstring id;
    LPWSTR id_str = nullptr;
    if (SUCCEEDED(device->GetId(&id_str))) {
      id = id_str;
      CoTaskMemFree(id_str);
    }
    std::wstring name;
    ComPtr<IPropertyStore> props;
    if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
      PROPVARIANT var;
      PropVariantInit(&var);
      if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &var)) && var.vt == VT_LPWSTR) {
        name = var.pwszVal;
      }
      PropVariantClear(&var);
    }
    if (visit(device.Get(), id, name)) {
      break;
    }
  }
  return S_OK;
}

HRESULT create_enumerator(ComPtr<IMMDeviceEnumerator> & enumerator)
{
  return CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
           IID_PPV_ARGS(&enumerator));
}

}  // namespace

std::optional<WasapiSampleFormat> wasapi_sample_format(SampleFormat format)
{
  switch (format) {
    case SampleFormat::Float:
      return WasapiSampleFormat{true, 32, 32};
    case SampleFormat::Float64:
      return WasapiSampleFormat{true, 64, 64};
    case SampleFormat::S16:
      return WasapiSampleFormat{false, 16, 16};
    case SampleFormat::S32:
      return WasapiSampleFormat{false, 32, 32};
    case SampleFormat::S24:
      // S24 is 24 significant bits in a 32-bit container
      return WasapiSampleFormat{false, 32, 24};
    default:
      return std::nullopt;
  }
}

struct WasapiProxyImpl::Impl
{
  ComPtr<IMMDevice> device;
  ComPtr<IAudioClient> client;          // activated; initialized once open() succeeds
  ComPtr<IAudioRenderClient> render;
  ComPtr<IAudioCaptureClient> capture;
  HANDLE event = nullptr;               // signaled by WASAPI when buffer space or data is ready
  AudioDirection direction = AudioDirection::Playback;
  UINT32 buffer_frames = 0;
  UINT32 bytes_per_frame = 0;
  unsigned int samplerate = 0;
  bool initialized = false;
  bool started = false;
  std::vector<uint8_t> pending;         // captured bytes not yet returned by read()

  void release()
  {
    render.Reset();
    capture.Reset();
    client.Reset();
    device.Reset();
    if (event) {
      CloseHandle(event);
      event = nullptr;
    }
    pending.clear();
    buffer_frames = 0;
    bytes_per_frame = 0;
    initialized = false;
    started = false;
  }
};

WasapiProxyImpl::WasapiProxyImpl()
: impl_(std::make_unique<Impl>()), error_str_(""), format_(SampleFormat::Unknown)
{}

WasapiProxyImpl::~WasapiProxyImpl()
{
  close();
}

bool WasapiProxyImpl::is_open() const
{
  return impl_->client != nullptr;
}

int WasapiProxyImpl::fail(const char * context, long hr)
{
  error_str_ = hresult_error(context, static_cast<HRESULT>(hr));
  printf("WasapiProxyImpl: %s\n", error_str_.c_str());
  return -1;
}

std::vector<std::string> WasapiProxyImpl::list_device_names(AudioDirection direction)
{
  ensure_com();
  std::vector<std::string> names;
  ComPtr<IMMDeviceEnumerator> enumerator;
  if (FAILED(create_enumerator(enumerator))) {
    return names;
  }
  for_each_endpoint(enumerator.Get(), to_data_flow(direction),
    [&names](IMMDevice *, const std::wstring &, const std::wstring & name) {
      names.push_back(to_utf8(name));
      return false;
    });
  return names;
}

std::optional<std::string> WasapiProxyImpl::resolve_device(
  const std::string & device,
  AudioDirection direction)
{
  impl_->device.Reset();
  ComPtr<IMMDeviceEnumerator> enumerator;
  HRESULT hr = create_enumerator(enumerator);
  if (FAILED(hr)) {
    return hresult_error("cannot create device enumerator", hr);
  }
  EDataFlow flow = to_data_flow(direction);

  if (device.empty() || device == "default") {
    hr = enumerator->GetDefaultAudioEndpoint(flow, eConsole, &impl_->device);
    if (FAILED(hr)) {
      return hresult_error(std::format("no default {} device", direction_name(direction)), hr);
    }
    return std::nullopt;
  }

  // Match an exact endpoint ID, otherwise the first friendly name containing the requested name
  std::wstring wanted = to_wide(device);
  std::wstring wanted_lower = to_lower(wanted);
  std::vector<std::string> available;
  ComPtr<IMMDevice> name_match;
  for_each_endpoint(enumerator.Get(), flow,
    [&](IMMDevice * endpoint, const std::wstring & id, const std::wstring & name) {
      available.push_back(to_utf8(name));
      if (id == wanted) {
        impl_->device = endpoint;
        return true;
      }
      if (!name_match && to_lower(name).find(wanted_lower) != std::wstring::npos) {
        name_match = endpoint;
      }
      return false;
    });
  if (FAILED(hr)) {
    // Report the real cause rather than "not found" with an empty device list
    return hresult_error(std::format("cannot enumerate {} devices", direction_name(direction)), hr);
  }
  if (!impl_->device) {
    impl_->device = name_match;
  }
  if (!impl_->device) {
    std::string list;
    for (const auto & name : available) {
      list += (list.empty() ? "'" : ", '") + name + "'";
    }
    return std::format("WASAPI: {} device '{}' not found. Available {} devices: {}",
             direction_name(direction), device, direction_name(direction),
             list.empty() ? "none" : list);
  }
  return std::nullopt;
}

std::optional<std::string> WasapiProxyImpl::preopen(
  const std::string & device,
  AudioDirection direction)
{
  ensure_com();
  if (impl_->client) {
    return std::nullopt;
  }
  device_name_ = device;
  impl_->direction = direction;
  auto resolve_error = resolve_device(device, direction);
  if (resolve_error) {
    error_str_ = *resolve_error;
    return error_str_;
  }
  HRESULT hr = impl_->device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
    reinterpret_cast<void **>(impl_->client.GetAddressOf()));
  if (FAILED(hr)) {
    impl_->release();
    error_str_ = hresult_error("cannot activate audio client", hr);
    return error_str_;
  }
  return std::nullopt;
}

std::optional<std::string> WasapiProxyImpl::open(
  DeviceParams & params,
  AudioDirection direction)
{
  ensure_com();
  printf("WasapiProxyImpl::open called at %s\n", format_timestamp().c_str());
  printf(
    "WasapiProxyImpl::open called with params: device=%s, channels=%u, samplerate=%u, format=%d, direction=%s\n",
           params.device.c_str(),
           params.channels,
           params.samplerate,
           static_cast<int>(params.format),
           direction_name(direction));
  error_str_.clear();

  if (params.channels == 0 || params.samplerate == 0) {
    error_str_ = std::format("WASAPI: invalid stream parameters: channels={}, samplerate={}",
        params.channels, params.samplerate);
    return error_str_;
  }

    // Reuse a pre-opened (activated, not yet initialized) client for the same device
  bool reuse = impl_->client && !impl_->initialized && device_name_ == params.device &&
    impl_->direction == direction;
  if (!reuse) {
    impl_->release();
    auto resolve_error = resolve_device(params.device, direction);
    if (resolve_error) {
      error_str_ = *resolve_error;
      return error_str_;
    }
  }
  device_name_ = params.device;
  impl_->direction = direction;

  const DWORD stream_flags = AUDCLNT_STREAMFLAGS_EVENTCALLBACK |
    AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY;
    // Buffer duration in 100 ns units
  const REFERENCE_TIME buffer_duration = static_cast<REFERENCE_TIME>(
    10000000.0 * params.buffer_frames / params.samplerate);

    // Sometimes a requested format fails but a different one succeeds (same order as ALSA).
    // An audio client can only be initialized once, so each attempt activates a new one.
  for (SampleFormat format : {params.format, SampleFormat::S32, SampleFormat::S16,
      SampleFormat::Float})
  {
    if (!error_str_.empty()) {
      printf("WasapiProxyImpl::open failed: (%s)\nretrying with format %d\n", error_str_.c_str(),
        static_cast<int>(format));
    }
    WAVEFORMATEXTENSIBLE wfx;
    if (!make_wave_format(format, params.channels, params.samplerate, wfx)) {
      error_str_ = std::format("WASAPI: unsupported sample format {}", static_cast<int>(format));
      continue;
    }
    if (!impl_->client) {
      HRESULT hr = impl_->device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
        reinterpret_cast<void **>(impl_->client.GetAddressOf()));
      if (FAILED(hr)) {
        // A device-level failure, so other formats will not help
        error_str_ = hresult_error("cannot activate audio client", hr);
        break;
      }
    }
    HRESULT hr = impl_->client->Initialize(AUDCLNT_SHAREMODE_SHARED, stream_flags,
      buffer_duration, 0, &wfx.Format, nullptr);
    if (FAILED(hr)) {
      error_str_ = hresult_error("cannot initialize audio client", hr);
      impl_->client.Reset();
      continue;
    }
    impl_->initialized = true;

    impl_->event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!impl_->event) {
      error_str_ = hresult_error("cannot create buffer event", HRESULT_FROM_WIN32(GetLastError()));
      break;
    }
    hr = impl_->client->SetEventHandle(impl_->event);
    if (FAILED(hr)) {
      error_str_ = hresult_error("cannot set buffer event", hr);
      break;
    }
    hr = impl_->client->GetBufferSize(&impl_->buffer_frames);
    if (FAILED(hr)) {
      error_str_ = hresult_error("cannot get buffer size", hr);
      break;
    }
    if (direction == AudioDirection::Playback) {
      hr = impl_->client->GetService(IID_PPV_ARGS(&impl_->render));
    } else {
      hr = impl_->client->GetService(IID_PPV_ARGS(&impl_->capture));
    }
    if (FAILED(hr)) {
      error_str_ = hresult_error("cannot get render/capture service", hr);
      break;
    }
    if (direction == AudioDirection::Capture) {
        // Capture runs continuously; playback starts once data is buffered (see write())
      hr = impl_->client->Start();
      if (FAILED(hr)) {
        error_str_ = hresult_error("cannot start capture", hr);
        break;
      }
      impl_->started = true;
    }

    impl_->bytes_per_frame = wfx.Format.nBlockAlign;
    impl_->samplerate = params.samplerate;
    error_str_.clear();
    if (format != params.format) {
      printf("\nWarning: Requested sample format %d not available, using %d instead.\n",
        static_cast<int>(params.format), static_cast<int>(format));
    }
    format_ = format;
    params.format = format;
    params.buffer_frames = impl_->buffer_frames;
    printf("WasapiProxyImpl::open succeeded at %s: buffer %u frames\n",
      format_timestamp().c_str(), impl_->buffer_frames);
    return std::nullopt;
  }

  impl_->release();
  if (error_str_.empty()) {
    error_str_ = "WASAPI: cannot open audio device";
  }
  return error_str_;
}

void WasapiProxyImpl::close()
{
  ensure_com();
  printf("WasapiProxyImpl::close called\n");
  if (impl_->initialized && impl_->client) {
    if (impl_->render) {
        // Drain: let queued playback finish, like snd_pcm_drain
      UINT32 padding = 0;
      if (SUCCEEDED(impl_->client->GetCurrentPadding(&padding)) && padding > 0) {
        if (!impl_->started && SUCCEEDED(impl_->client->Start())) {
          impl_->started = true;
        }
        int drain_ms = static_cast<int>(1000LL * impl_->buffer_frames /
          std::max(impl_->samplerate, 1u)) + DRAIN_MARGIN_MS;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(drain_ms);
        while (impl_->started && padding > 0 && std::chrono::steady_clock::now() < deadline) {
          WaitForSingleObject(impl_->event, 10);
          if (FAILED(impl_->client->GetCurrentPadding(&padding))) {
            break;
          }
        }
      }
    }
    if (impl_->started) {
      impl_->client->Stop();
    }
  }
  impl_->release();
}

int WasapiProxyImpl::write(
  int samples,
  void * data,
  int channels,
  std::atomic<bool> * shutdown_flag
)
{
  ensure_com();
  if (!impl_->render) {
    error_str_ = "WASAPI playback device is not opened";
    return -1;
  }
  if (channels <= 0) {
    return -2;
  } else if (samples <= 0) {
    return 0;
  }
  const UINT32 frames = static_cast<UINT32>(samples / channels);
  const UINT32 bytes_per_frame = impl_->bytes_per_frame;
  const uint8_t * src = static_cast<const uint8_t *>(data);
    // Start playback once half the buffer is filled, like ALSA's start threshold
  const UINT32 start_threshold = impl_->buffer_frames / 2;

  UINT32 written = 0;
  while (written < frames) {
    if (shutdown_flag && shutdown_flag->load()) {
      break;
    }
    UINT32 padding = 0;
    HRESULT hr = impl_->client->GetCurrentPadding(&padding);
    if (FAILED(hr)) {
      return fail("cannot get buffer padding", hr);
    }
    if (!impl_->started && padding >= start_threshold) {
      hr = impl_->client->Start();
      if (FAILED(hr)) {
        return fail("cannot start playback", hr);
      }
      impl_->started = true;
    }
    UINT32 available = impl_->buffer_frames - padding;
    if (available == 0) {
      WaitForSingleObject(impl_->event, EVENT_WAIT_MS);
      continue;
    }
    UINT32 count = std::min(available, frames - written);
    BYTE * buffer = nullptr;
    hr = impl_->render->GetBuffer(count, &buffer);
    if (FAILED(hr)) {
      return fail("cannot get render buffer", hr);
    }
    std::memcpy(buffer, src + static_cast<size_t>(written) * bytes_per_frame,
      static_cast<size_t>(count) * bytes_per_frame);
    hr = impl_->render->ReleaseBuffer(count, 0);
    if (FAILED(hr)) {
      return fail("cannot release render buffer", hr);
    }
    written += count;
  }
  return static_cast<int>(written) * channels;
}

int WasapiProxyImpl::read(
  int samples,
  void * data,
  int channels,
  SampleFormat format,
  std::atomic<bool> * shutdown_flag
)
{
  (void)format;  // the device was opened with format_, which callers read in
  ensure_com();
  if (!impl_->capture) {
    error_str_ = "WASAPI capture device is not opened";
    return -1;
  }
  if (channels <= 0) {
    return -2;
  } else if (samples <= 0) {
    return 0;
  }
  const UINT32 frames = static_cast<UINT32>(samples / channels);
  const UINT32 bytes_per_frame = impl_->bytes_per_frame;
  uint8_t * dst = static_cast<uint8_t *>(data);
  UINT32 got = 0;

    // Frames left over from the previous packet come first
  if (!impl_->pending.empty()) {
    UINT32 count = std::min(static_cast<UINT32>(impl_->pending.size() / bytes_per_frame), frames);
    size_t bytes = static_cast<size_t>(count) * bytes_per_frame;
    std::memcpy(dst, impl_->pending.data(), bytes);
    impl_->pending.erase(impl_->pending.begin(), impl_->pending.begin() + bytes);
    got = count;
  }

  while (got < frames) {
    if (shutdown_flag && shutdown_flag->load()) {
      break;
    }
    UINT32 packet_frames = 0;
    HRESULT hr = impl_->capture->GetNextPacketSize(&packet_frames);
    if (FAILED(hr)) {
      return fail("cannot get capture packet size", hr);
    }
    if (packet_frames == 0) {
      WaitForSingleObject(impl_->event, EVENT_WAIT_MS);
      continue;
    }
    BYTE * buffer = nullptr;
    UINT32 count = 0;
    DWORD flags = 0;
    hr = impl_->capture->GetBuffer(&buffer, &count, &flags, nullptr, nullptr);
    if (FAILED(hr)) {
      return fail("cannot get capture buffer", hr);
    }
    const bool silent = (flags & AUDCLNT_BUFFERFLAGS_SILENT) != 0;
    UINT32 use = std::min(count, frames - got);
    uint8_t * out = dst + static_cast<size_t>(got) * bytes_per_frame;
    if (silent) {
      std::memset(out, 0, static_cast<size_t>(use) * bytes_per_frame);
    } else {
      std::memcpy(out, buffer, static_cast<size_t>(use) * bytes_per_frame);
    }
    if (count > use) {
      size_t extra = static_cast<size_t>(count - use) * bytes_per_frame;
      if (silent) {
        impl_->pending.insert(impl_->pending.end(), extra, 0);
      } else {
        const uint8_t * rest = buffer + static_cast<size_t>(use) * bytes_per_frame;
        impl_->pending.insert(impl_->pending.end(), rest, rest + extra);
      }
    }
    hr = impl_->capture->ReleaseBuffer(count);
    if (FAILED(hr)) {
      return fail("cannot release capture buffer", hr);
    }
    got += use;
  }
  return static_cast<int>(got) * channels;
}
