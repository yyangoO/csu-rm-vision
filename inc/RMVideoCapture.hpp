#ifndef RMVIDEOCAPTURE_H
#define RMVIDEOCAPTURE_H


#include "opencv2/core.hpp"

class RMVideoCapture {
public:
    RMVideoCapture(const char * device, int size_buffer = 1);
    ~RMVideoCapture();
    bool startStream();
    bool closeStream();
    bool setExposureTime(bool auto_exp, int t);
    bool setVideoFormat(unsigned int width, unsigned int height, bool mjpg = 1);
    bool changeVideoFormat(int width, int height, bool mjpg = 1);
    bool getVideoSize(int & width, int & height);

    bool setVideoFPS(int fps);
    bool setBufferSize(unsigned int bsize);
    void restartCapture();
    int getFrameCount(){
        return cur_frame;
    }

    void info();

    RMVideoCapture& operator >> (cv::Mat & image);

private:
    void cvtRaw2Mat(const void * data, cv::Mat & image);
    bool refreshVideoFormat();
    bool initMMap();
    int xioctl(int fd, int request, void *arg);

private:
    struct MapBuffer {
        void * ptr;
        unsigned int size;
    };
    unsigned int capture_width;
    unsigned int capture_height;
    unsigned int format;
    int fd;
    unsigned int buffer_size;
    unsigned int buffr_idx;
    unsigned int cur_frame;
    MapBuffer * mb;
    const char * video_path;
};


#endif // RMVIDEOCAPTURE_H
