/*
 * @ File name: rin_proc_lib.cpp
 * @ Effect: My persional ways to process images and add threads.(Based on Vision by SEU).
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include <math.h>
#include <iostream>
#include "sys/time.h"
#include "inc/rin_proc_lib.h"


using namespace std;
using namespace cv;


void ImgPorcCon::init(void)
{
    // Hardware initialization.
    _rin_serial.serrial_cmd();

    camera_reopen:
    try
    {
        _mono_cap.cap_open("/dev/video0", 3);
    }
    catch(exception e)
    {
        cout << e.what() << endl;
        goto camera_reopen;
    }
    // parameters and functions initialization.
    _params.param_read();
    _rin_KF.init();

    // Functions's run parameters initialization.
    _rin_serial.vision_init();
    _params.robo_cmd.init_flag = _rin_serial.robo_data.enemy_color;
    _params.robo_cmd.set_flag = _rin_serial.robo_data.set_flag;

    if((_params.robo_cmd.init_flag == true) && (_params.robo_cmd.set_flag == false))
    {
        _params.robo_cmd.init_flag = false;
        _params.robo_cmd.set_flag = false;
        _params.robo_cmd.enemy_color = _rin_serial.robo_data.enemy_color;
        _params.robo_cmd.debug_flag = _rin_serial.robo_data.debug_flag;
        _params.robo_cmd.aim_or_rune = _rin_serial.robo_data.aim_or_rune;
        _params.robo_cmd.armor_type = _rin_serial.robo_data.armor_type;
    }
    _armor_mono.frame_info.frame_reso_x = _params.mono_cam_val.mono_cam_far_resolution_x;
    _armor_mono.frame_info.frame_reso_y = _params.mono_cam_val.mono_cam_far_resolution_x;
    _armor_mono.frame_info.frame_mid_x = _params.mono_cam_val.mono_cam_far_middle_x;
    _armor_mono.frame_info.frame_mid_y = _params.mono_cam_val.mono_cam_far_middle_y;
    _rune_mono.frame_info.frame_reso_x = _params.mono_cam_val.mono_cam_far_resolution_x;
    _rune_mono.frame_info.frame_reso_y = _params.mono_cam_val.mono_cam_far_resolution_x;
    _rune_mono.frame_info.frame_mid_x = _params.mono_cam_val.mono_cam_far_middle_x;
    _rune_mono.frame_info.frame_mid_y = _params.mono_cam_val.mono_cam_far_middle_y;
    // Hardware set and run.
    _mono_cap.set_format(_params.mono_cam_val.mono_cam_far_resolution_x, \
                         _params.mono_cam_val.mono_cam_far_resolution_y, 1);
    _mono_cap.set_FPS(_params.mono_cam_val.mono_cam_far_FPS);
    _mono_cap.set_exposure_time(false, _params.mono_cam_val.mono_cam_exp_val_l);
    _mono_cap.start_stream();
}

void ImgPorcCon::info_get(void)
{
    struct timeval tv;
    struct timezone tz;
    image_reget:
    try
    {
        _mono_cap >> _mono_img;
        gettimeofday(&tv, &tz);
//        cout << tv.tv_usec << "image catching..." << endl;
    }
    catch(exception e)
    {
        cout << "Error: image catch failed, capture reopen." << e.what() << endl;
        _mono_cap.restart_capture();
        goto image_reget;
    }
    if(_mono_img.empty())
    {
        cout << "Warnning: camera image empty, capture reopen." << endl;
        _mono_cap.restart_capture();
        goto image_reget;
    }
}

void ImgPorcCon::img_proc(void)
{
    _rin_serial.msg_read();
    _params.robo_cmd.init_flag = _rin_serial.robo_data.init_flag;
    _params.robo_cmd.set_flag = _rin_serial.robo_data.set_flag;
    if((_params.robo_cmd.init_flag == false) &&
        (_params.robo_cmd.set_flag == true))
    {
        _params.robo_cmd.init_flag = false;
        _params.robo_cmd.set_flag = false;
        _params.robo_cmd.debug_flag = _rin_serial.robo_data.debug_flag;
        _params.robo_cmd.enemy_color = _rin_serial.robo_data.enemy_color;
        _params.robo_cmd.aim_or_rune = _rin_serial.robo_data.aim_or_rune;
        _params.robo_cmd.armor_type = _rin_serial.robo_data.armor_type;
    }
    if(_params.robo_cmd.debug_flag == true)
    {
        _params.param_read();

        _armor_mono.frame_info.frame_reso_x = _params.mono_cam_val.mono_cam_far_resolution_x;
        _armor_mono.frame_info.frame_reso_y = _params.mono_cam_val.mono_cam_far_resolution_x;
        _armor_mono.frame_info.frame_mid_x = _params.mono_cam_val.mono_cam_far_middle_x;
        _armor_mono.frame_info.frame_mid_y = _params.mono_cam_val.mono_cam_far_middle_y;
        _rune_mono.frame_info.frame_reso_x = _params.mono_cam_val.mono_cam_far_resolution_x;
        _rune_mono.frame_info.frame_reso_y = _params.mono_cam_val.mono_cam_far_resolution_x;
        _rune_mono.frame_info.frame_mid_x = _params.mono_cam_val.mono_cam_far_middle_x;
        _rune_mono.frame_info.frame_mid_y = _params.mono_cam_val.mono_cam_far_middle_y;

        _mono_cap.set_exposure_time(false, _params.mono_cam_val.mono_cam_exp_val_l);
    }
    // Image process.
    if(_params.robo_cmd.aim_or_rune == RIN_AIM)
        _armor_mono.armor_mono_proc(_mono_img, _params);
    else
        _rune_mono.rune_mono_proc(_mono_img, _params);
}

void ImgPorcCon::robo_cmd(void)
{
    if(_params.robo_cmd.aim_or_rune == RIN_AIM)
    {
        _rin_serial._pc_data.X_offset = _armor_mono.target_info.X_offset;
        _rin_serial._pc_data.Y_offset = _armor_mono.target_info.Y_offset;
        _rin_serial._pc_data.Z_offset = _armor_mono.target_info.Z_offset;
        if(_armor_mono.target_info.target_flag == true)
            _rin_serial._pc_data.target_flag = 0xff;
        else
            _rin_serial._pc_data.target_flag = 0x00;
    }
    else
    {
        _rin_serial._pc_data.X_offset = _rune_mono.rune_target_info.X_offset;
        _rin_serial._pc_data.Y_offset = _rune_mono.rune_target_info.Y_offset;
        _rin_serial._pc_data.Z_offset = 0;
        if(_rune_mono.rune_target_info.target_flag == true)
            _rin_serial._pc_data.target_flag = 0xff;
        else
            _rin_serial._pc_data.target_flag = 0x00;
    }
    _rin_serial.msg_send();
}

void ImgPorcCon::vision_run(void)
{
    info_get();
    img_proc();
    robo_cmd();
}


// Arm to fill the lightbar's hole.
// Input image: Bin; Output image: Bin.
void fill_hole(Mat &in_img, Mat &out_img)
{
    Mat org_img = in_img;
    Mat cut_img;
    Size size = org_img.size();
    Mat temp = Mat::zeros(size.height + 2, size.width + 2, org_img.type());
    org_img.copyTo(temp(Range(1, size.height + 1), Range(1, size.width + 1)));
    floodFill(temp, Point(0, 0), Scalar(255));
    temp(Range(1, size.height + 1), Range(1, size.width + 1)).copyTo(cut_img);
    out_img = org_img | (~cut_img);
}

// Calibration
void calibration_img_get(void)
{
    int idx = 0;
    RinVideoCapture mono_cap;
    mono_cap.cap_open("/dev/video0", 3);
    Mat img;
    string path;
    mono_cap.set_format(1280, 720, 1);
    mono_cap.set_FPS(60);
    mono_cap.set_exposure_time(false, 8);
    mono_cap.start_stream();
    while(1)
    {
        mono_cap >> img;
//        cvNamedWindow("calibration_image", CV_WINDOW_AUTOSIZE);
//        imshow("calibration_image", img);
        if(waitKey(1) != -1)
        {
            idx++;
            path = "/home/csu-rm-infantry-1/csu_rm_vision/vision_by_rinck/csu_rm_vision_v2.1/data/";
            path = path + to_string(idx);
            path = path + ".jpg";
            imwrite(path, img);
        }
    }
}

// Kalman filter measurement update function
void RinKalmanFilter::init(void)
{
    _dt = 0.0f;
    _F_ = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 400.0f , 0.0f, \
                                    0.0f, 1.0f, 0.0f, 400.0f , \
                                    0.0f, 0.0f, 1.0f, 0.0f, \
                                    0.0f, 0.0f, 0.0f, 1.0f);
    _U_ = (cv::Mat_<float>(4, 1) << 0.0f, \
                                    0.0f, \
                                    0.0f, \
                                    0.0f);
    _P_ = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f , 0.0f, \
                                     0.0f, 1.0f, 0.0f , 0.0f, \
                                     0.0f, 0.0f, 50.0f, 0.0f, \
                                     0.0f, 0.0f, 0.0f , 50.0f);
    _Q_ = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, 0.0f, \
                                    0.0f, 1.0f, 0.0f, 0.0f, \
                                    0.0f, 0.0f, 1.0f, 0.0f, \
                                    0.0f, 0.0f, 0.0f, 1.0f);
    _H_ = (cv::Mat_<float>(2, 4) << 1.0f, 0.0f, 0.0f, 0.0f, \
                                    0.0f, 1.0f, 0.0f, 0.0f);
    _R_ = (cv::Mat_<float>(2, 2) << 2000.0f, 0.0f, \
                                    0.0f, 2000.0f);
    _I_ = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, 0.0f, \
                                    0.0f, 1.0f, 0.0f, 0.0f, \
                                    0.0f, 0.0f, 1.0f, 0.0f, \
                                    0.0f, 0.0f, 0.0f, 1.0f);
}
void RinKalmanFilter::prediction(void)
{
    Mat Ft;
    _x_ = _F_* _x_ + _U_;
    transpose(_F_, Ft);
    _P_ = _F_ * _P_ * Ft + _Q_;
}

void RinKalmanFilter::measurement_upt(const Mat &z)
{
    Mat y, Ht, S, Si, K;
    transpose(_H_, Ht);
    y = z - _H_ * _x_;
    S = _H_ * _P_ * Ht + _R_;
    invert(S, Si);
    K = _P_ * Ht * Si;
    _x_ = _x_ + K * y;
    _P_ = (_I_ - K * _H_) * _P_;
}
