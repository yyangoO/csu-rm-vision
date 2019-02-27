#ifndef DEVICE_H
#define DEVICE_H


#include "inc/includes.h"
#include "inc/parameters.h"


class DeviceCfg
{
public:
    void mono_cam_init(void);
    void mono_cam_cfg(void);
    void bino_cam_init(void);
    void bino_cam_cfg(void);
    void serrial_init(void);
    void serrial_i(void);
    void serrial_o(void);
public:
    void mono_undistort(cv::Mat &in_img, cv::Mat &out_img, Params params);
};


extern DeviceCfg dev_cfg;


#endif // DEVICE_H
