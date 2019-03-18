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
    _rin_serial.serrial_cmd();
    _rin_serial.robo_data.enemy_color = RIN_ENEMY_RED;
    _rin_serial.robo_data.reso_flag = RIN_RESO_CLOSE;
//    _rin_serial.msg_read();
    _params.param_read();
    _rin_KF.init();
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
    // Kalman filter.
    Mat x_in, z;
    float speed[2];
    struct timeval tv1, tv2;
    struct timezone tz1, tz2;
    curr_offset[0] = _armor_mono.target_info.X_offset;
    curr_offset[1] = _armor_mono.target_info.Y_offset;
    gettimeofday(&tv1, &tz1);
    proc_tim = tv2.tv_sec * 1000 + tv2.tv_usec / 1000 - \
               tv1.tv_sec * 1000 - tv1.tv_usec / 1000;
    gettimeofday(&tv2, &tz2);
    speed[0] = (curr_offset[0] - last_offset[0]) / 10;
    speed[1] = (curr_offset[1] - last_offset[1]) / 10;
    x_in = (Mat_<float>(4, 1) << curr_offset[0], \
                                 curr_offset[1], \
                                 speed[0],       \
                                 speed[1]);
    z = (Mat_<float>(2, 1) << curr_offset[0], \
                              curr_offset[1]);
    _rin_KF.get_info(x_in, proc_tim);
    _rin_KF.prediction();
    _rin_KF.measurement_upt(z);
    last_offset[0] = curr_offset[0];
    last_offset[1] = curr_offset[1];
    // Serial CMD.
    _rin_serial._pc_data.X_offset = _armor_mono.target_info.X_offset;
    _rin_serial._pc_data.Y_offset = _armor_mono.target_info.Y_offset;
    _rin_serial._pc_data.Z_offset = _armor_mono.target_info.Z_offset;
    _rin_serial._pc_data.X_KF = (int16_t)_rin_KF._x_.at<float>(0);
    _rin_serial._pc_data.Y_KF = (int16_t)_rin_KF._x_.at<float>(1);
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
                                     0.0f, 0.0f, 100.0f, 0.0f, \
                                     0.0f, 0.0f, 0.0f , 100.0f);
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
//    cout <<endl << "second:" << endl;
//    for(int idx = 0; idx < Ft.rows; idx++)
//    {
//        for(int c_idx = 0; c_idx < Ft.cols; c_idx++)
//        {
//            cout << Ft.at<float>(idx, c_idx) << " ";
//        }
//        cout << endl;
//    }
//    cout << endl;
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
