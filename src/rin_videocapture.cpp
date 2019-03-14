/*
 * @ File name: rin_videocapture.cpp
 * @ Effect: Video Capture with no delay (Based on:RMVideoCapture, by DJI)
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/3/11
 */


#include "inc/rin_videocapture.h"
#include <linux/videodev2.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;

RinVideoCapture::RinVideoCapture(const char * device, int buffer_size) : _video_path(device)
{
    _fd = open(device, O_RDWR);
    _buffer_size = buffer_size;
    _buffr_idx = 0;
    _curr_frame = 0;
    _capture_width = 0;
    _capture_height = 0;
    _mb = new MapBuffer_t[_buffer_size];
}

void RinVideoCapture::restart_capture(void)
{
    close(_fd);
    _fd = open(_video_path, O_RDWR);
    _buffr_idx = 0;
    _curr_frame = 0;
}

RinVideoCapture::~RinVideoCapture(void)
{
    close(_fd);
    delete [] _mb;
}

void RinVideoCapture::cvt_Raw2Mat(const void * data, Mat & img)
{
    if (_format == V4L2_PIX_FMT_MJPEG)
    {
        Mat src(_capture_height, _capture_width, CV_8UC3, (void*) data);
        img = imdecode(src, 1);
    }
    else if(_format == V4L2_PIX_FMT_YUYV)
    {
        Mat yuyv(_capture_height, _capture_width, CV_8UC2, (void*) data);
        cvtColor(yuyv, img, CV_YUV2BGR_YUYV);
    }
}

RinVideoCapture & RinVideoCapture::operator >> (cv::Mat & img)
{
    struct v4l2_buffer bufferinfo = {};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = _buffr_idx;
    ioctl(_fd, VIDIOC_DQBUF, &bufferinfo);
    cvt_Raw2Mat(_mb[_buffr_idx].ptr, img);
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = _buffr_idx;
    ioctl(_fd, VIDIOC_QBUF, &bufferinfo);
    ++_buffr_idx;
    _buffr_idx = _buffr_idx >= _buffer_size ? _buffr_idx - _buffer_size : _buffr_idx;
    ++_curr_frame;
    return *this;
}

bool RinVideoCapture::init_MMap(void)
{
    struct v4l2_requestbuffers bufrequest = {};
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = _buffer_size;
    ioctl(_fd, VIDIOC_REQBUFS, &bufrequest);

    for(unsigned int idx = 0; idx < _buffer_size; ++idx)
    {
        struct v4l2_buffer bufferinfo = {};
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = idx;
        ioctl(_fd, VIDIOC_QUERYBUF, &bufferinfo);
        _mb[idx].ptr = mmap(
            NULL,
            bufferinfo.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            _fd,
            bufferinfo.m.offset);
        _mb[idx].size = bufferinfo.length;
        if(_mb[idx].ptr == MAP_FAILED)
        {
            return false;
        }
        memset(_mb[idx].ptr, 0, bufferinfo.length);
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = idx;
        if(ioctl(_fd, VIDIOC_QBUF, &bufferinfo) < 0)
        {
            return false;
        }
    }
    return true;
}

bool RinVideoCapture::start_stream(void)
{
    _curr_frame = 0;
    refresh_format();
    if(init_MMap() == false)
    {
        return false;
    }
    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(_fd, VIDIOC_STREAMON, &type) < 0)
    {
        return false;
    }
    return true;
}

bool RinVideoCapture::close_stream(void)
{
    _curr_frame = 0;
    _buffr_idx = 0;
    __u32 type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(_fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        return false;
    }
    for(unsigned int idx = 0; idx < _buffer_size; ++idx)
    {
        munmap(_mb[idx].ptr, _mb[idx].size);
    }
    return true;
}

bool RinVideoCapture::set_exposure_time(bool auto_exp, int t)
{
    if (auto_exp)
    {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_AUTO;
        if(rin_ioctl(_fd, VIDIOC_S_CTRL, &control_s) < 0)
        {
            return false;
        }
    }
    else
    {
        struct v4l2_control control_s;
        control_s.id = V4L2_CID_EXPOSURE_AUTO;
        control_s.value = V4L2_EXPOSURE_MANUAL;
        if(rin_ioctl(_fd, VIDIOC_S_CTRL, &control_s) < 0)
        {
            return false;
        }

        control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        control_s.value = t;
        if(rin_ioctl(_fd, VIDIOC_S_CTRL, &control_s) < 0)
        {
            return false;
        }
    }
    return true;
}

bool RinVideoCapture::chg_format(int width, int height, bool mjpg)
{
    close_stream();
    restart_capture();
    set_format(width, height, mjpg);
    start_stream();
    return true;
}

bool RinVideoCapture::set_format(unsigned int width, unsigned int height, bool mjpg)
{
    if (_capture_width == width && _capture_height == height)
    {
        return true;
    }
    _capture_width = width;
    _capture_height = height;
    _curr_frame = 0;
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    if (mjpg == true)
    {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    }
    else
    {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    }
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (-1 == rin_ioctl(_fd, VIDIOC_S_FMT, &fmt))
    {
        return false;
    }
    return true;
}

bool RinVideoCapture::refresh_format(void)
{
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == rin_ioctl(_fd, VIDIOC_G_FMT, &fmt))
    {
        return false;
    }
    _capture_width = fmt.fmt.pix.width;
    _capture_height = fmt.fmt.pix.height;
    _format = fmt.fmt.pix.pixelformat;
    return true;
}


bool RinVideoCapture::get_size(int & width, int & height)
{
    if (_capture_width == 0 || _capture_height == 0)
    {
        if (refresh_format() == false)
        {
            return false;
        }
    }
    width = _capture_width;
    height = _capture_height;
    return true;
}

bool RinVideoCapture::set_FPS(int fps)
{
    struct v4l2_streamparm stream_param = {};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.denominator = fps;
    stream_param.parm.capture.timeperframe.numerator = 1;

    if (-1 == rin_ioctl(_fd, VIDIOC_S_PARM, &stream_param))
    {
        return false;
    }
    return true;
}

bool RinVideoCapture::set_buffersize(unsigned int bsize)
{
    if (_buffer_size != bsize){
        _buffer_size = bsize;
        delete [] _mb;
        _mb = new MapBuffer_t[_buffer_size];
    }
    return true;
}

float RinVideoCapture::get_FPS(void)
{
    struct v4l2_streamparm stream_param = {};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(_fd, VIDIOC_G_PARM, &stream_param))
    {
        return false;
    }
    return ((float)stream_param.parm.capture.timeperframe.denominator / \
            (float)stream_param.parm.capture.timeperframe.numerator);
}

int RinVideoCapture::rin_ioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}
