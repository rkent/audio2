#ifndef SNDPLAY_BUFFER_FILE_HPP
#define SNDPLAY_BUFFER_FILE_HPP
#include <sndfile.h>
#include <vector>
#include <memory>
#include <atomic>
#include "boost/lockfree/spsc_queue.hpp"

#include "sndplay/alsaops.hpp"

// Virtual I/O context for reading/writing from/to memory
typedef struct {
    const unsigned char *data = nullptr;
    sf_count_t length = 0;
    sf_count_t offset = 0;
    sf_count_t capacity = 0;
} VIO_DATA;

typedef struct {
    VIO_DATA vio_data;
    SNDFILE * sndfile = nullptr;
    SF_INFO sfinfo;
} VIO_SOUNDFILE;

typedef struct {
    std::shared_ptr<std::vector<unsigned char>> file_data;
    AlsaHwParams hw_vals;
    AlsaSwParams sw_vals;
} PlayBufferParams;

int open_sndfile_from_buffer(VIO_SOUNDFILE & vio_sndfile, int mode);

int write_buffer(void* buffer, int format, int frames, int channels, int sample_rate);

PlayBufferParams get_file(char * file_path);

void play_buffer_thread(boost::lockfree::spsc_queue<PlayBufferParams>* audio_queue, std::atomic<bool>* shutdown_flag, std::atomic<bool>* data_available);

int sfg_read(SNDFILE * sndfile, SF_INFO * sfinfo, void * buffer, int samples);
int sample_size_from_format(int format);
int sfg_write(SNDFILE * sndfile, void * buffer, int format, int samples);
#endif // SNDPLAY_BUFFER_FILE_HPP
