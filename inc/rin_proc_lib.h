/*
 * @ File name: rin_proc_lib.h
 * @ Effect: My persional ways to process images and add threads.(Based on Vision by SEU).
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef RIN_PROC_LIB_H
#define RIN_PROC_LIB_H


#include <mutex>
#include <string>
#include <vector>
#include "inc/includes.h"
#include "inc/parameters.h"
#include "inc/rin_videocapture.h"
#include "inc/armor_monocular.h"
#include "inc/rune_monocular.h"
#include "inc/parameters.h"
#include "inc/device.h"


class RinKalmanFilter {
public:
    void init(void);
    void get_info(cv::Mat x_in, float dt) {
        _x_ = x_in;
        _dt = dt;
    }
    void prediction(void);
    void measurement_upt(const cv::Mat &z);
public:
    float _dt;
    cv::Mat _x_;
    cv::Mat _F_;
    cv::Mat _U_;
    cv::Mat _P_;
    cv::Mat _Q_;
    cv::Mat _H_;
    cv::Mat _R_;
    cv::Mat _I_;
};

class ImgPorcCon
{
private:
    RinVideoCapture _mono_cap;
public:
    RinSerial _rin_serial;
private:
    Params _params;
    ArmorMono _armor_mono;
    RuneMono _rune_mono;
    RinKalmanFilter _rin_KF;
public:
    void init(void);
    void info_get(void);
    void img_proc(void);
    void robo_cmd(void);
    void vision_run(void);
private:
    cv::Mat _mono_img;
    float curr_offset[2];
    float last_offset[2];
    float proc_tim;
};


#endif // RIN_PROC_LIB_H
