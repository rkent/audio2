/**
 * @file list_devices.cpp
 * @brief Print the audio devices RtAudio finds, using its device-probing API.
 */

#include <iostream>

#include <RtAudio.h>

namespace
{

std::string format_names(RtAudioFormat formats)
{
  std::string result;
  auto append = [&result](const char * name) {
      if (!result.empty()) {result += " ";}
      result += name;
    };

  if (formats & RTAUDIO_SINT8) {append("SINT8");}
  if (formats & RTAUDIO_SINT16) {append("SINT16");}
  if (formats & RTAUDIO_SINT24) {append("SINT24");}
  if (formats & RTAUDIO_SINT32) {append("SINT32");}
  if (formats & RTAUDIO_FLOAT32) {append("FLOAT32");}
  if (formats & RTAUDIO_FLOAT64) {append("FLOAT64");}
  return result;
}

}  // namespace

int main()
{
  RtAudio audio;

  std::vector<unsigned int> device_ids = audio.getDeviceIds();
  if (device_ids.empty()) {
    std::cout << "No audio devices found.\n";
    return 0;
  }

  for (unsigned int id : device_ids) {
    RtAudio::DeviceInfo info = audio.getDeviceInfo(id);

    std::cout << "Device " << info.ID << ": " << info.name << "\n";
    std::cout << "  Output channels: " << info.outputChannels << "\n";
    std::cout << "  Input channels: " << info.inputChannels << "\n";
    std::cout << "  Duplex channels: " << info.duplexChannels << "\n";
    std::cout << "  Default output: " << (info.isDefaultOutput ? "yes" : "no") << "\n";
    std::cout << "  Default input: " << (info.isDefaultInput ? "yes" : "no") << "\n";
    std::cout << "  Native formats: " << format_names(info.nativeFormats) << "\n";
    std::cout << "  Preferred sample rate: " << info.preferredSampleRate << " Hz\n";
    std::cout << "  Supported sample rates:";
    for (unsigned int rate : info.sampleRates) {
      std::cout << " " << rate;
    }
    std::cout << "\n\n";
  }

  return 0;
}
