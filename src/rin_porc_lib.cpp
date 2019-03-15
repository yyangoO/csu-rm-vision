/*
 * @ File name: rin_proc_lib.cpp
 * @ Effect: My persional ways to process images and add threads.(Based on Vision by SEU).
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include <math.h>
//#include <mutex>
//#include <type_traits>
//#include <signal.h>
//#include <chrono>
//#include <thread>
//#include <memory>
#include <iostream>
#include "inc/rin_proc_lib.h"


using namespace std;
using namespace cv;


void ImgPorcCon::init(void)
{
    _rin_serial.serrial_cmd();
    _rin_serial.robo_data.enemy_color = RIN_ENEMY_RED;
    _rin_serial.robo_data.reso_flag = RIN_RESO_CLOSE;
//    _rin_serial.msg_read();
    _params.param_read();
    _params.BR_HSV_range.enemy_color = _rin_serial.robo_data.enemy_color;
    _mono_cap.cap_open("/dev/video0", 3);
    if(_rin_serial.robo_data.reso_flag == RIN_RESO_CLOSE)
    {
        _mono_cap.set_format(_params.mono_cam_val.mono_cam_close_resolution_x, \
                             _params.mono_cam_val.mono_cam_close_resolution_y, 1);
        _mono_cap.set_FPS(_params.mono_cam_val.mono_cam_close_FPS);
    }
    else
    {
        _mono_cap.set_format(_params.mono_cam_val.mono_cam_far_resolution_x, \
                             _params.mono_cam_val.mono_cam_far_resolution_y, 1);
        _mono_cap.set_FPS(_params.mono_cam_val.mono_cam_far_FPS);
    }
    _mono_cap.set_exposure_time(false, _params.mono_cam_val.mono_cam_exp_val_l);
    _mono_cap.start_stream();
}

void ImgPorcCon::info_get(void)
{
    _mono_cap >> _mono_img;
}

void ImgPorcCon::img_proc(void)
{
    _armor_mono.armor_mono_proc(_mono_img, _params);
}

void ImgPorcCon::robo_cmd(void)
{
    _rin_serial._pc_data.X_offset = (int16_t)(_armor_mono.target_info.X_offset);
    _rin_serial._pc_data.Y_offset = (int16_t)(_armor_mono.target_info.Y_offset);
    _rin_serial._pc_data.Z_offset = (int16_t)(_armor_mono.target_info.Z_offset);
//    _rin_serial.msg_read();
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
    Mat temp = Mat::zeros(size.height + 2, size.width + 2, org_img.type()); // Make image bigger.

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
    mono_cap.set_format(MONO_IMAGE_X_SIZE, MONO_IMAGE_Y_SIZE, 1);
    mono_cap.set_FPS(FPS);
    mono_cap.set_exposure_time(false, 8);
    mono_cap.start_stream();
    while(1)
    {
        mono_cap >> img;
        cvNamedWindow("calibration_image", CV_WINDOW_AUTOSIZE);
        imshow("calibration_image", img);
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
