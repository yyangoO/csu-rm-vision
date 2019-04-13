/*
 * @ File name: rin_videocapture.h
 * @ Effect: Video Capture with no delay (Based on:RMVideoCapture, by DJI)
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/3/11
 */


#ifndef RIN_VIDEOCAPTURE_H
#define RIN_VIDEOCAPTURE_H


#include "opencv2/core.hpp"


class RinVideoCapture {
public:
    void cap_open(const char* device, int buffer_size = 1);
    void cap_close(void);
    bool start_stream(void);
    bool close_stream(void);
    bool set_exposure_time(bool auto_exp, int t);
    bool set_format(unsigned int width, unsigned int height, bool mjpg = 1);
    bool chg_format(int width, int height, bool mjpg = 1);
    bool set_FPS(int fps);
    bool set_buffersize(unsigned int bsize);
    bool get_size(int & width, int & height);
    float get_FPS(void);
    void restart_capture(void);

    RinVideoCapture& operator >> (cv::Mat & img);

private:
    void cvt_Raw2Mat(const void * data, cv::Mat & img);
    bool refresh_format(void);
    bool init_MMap(void);
    int rin_ioctl(int fd, int request, void *arg);

private:
    struct MapBuffer_t {
        void * ptr;
        unsigned int size;
    };
    unsigned int _capture_width;
    unsigned int _capture_height;
    unsigned int _format;
    int _fd;
    unsigned int _buffer_size;
    unsigned int _buffr_idx;
    unsigned int _curr_frame;
    MapBuffer_t* _mb;
    const char* _video_path;
};


#endif // RIN_VIDEOCAPTURE_H
