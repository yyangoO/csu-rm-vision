/*
 * @ File name: parameters.cpp
 * @ Effect: Arm to manage the system's paramsters.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/8
 */


#include <iostream>

#include "inc/includes.h"
#include "inc/parameters.h"


using namespace std;
using namespace cv;


Params params;


// Read parameters from paramsters.xml.
void Params::cam_params_xml_read(void)
{
    FileStorage cam_params_fs(cam_params_xml_dir, FileStorage::READ);
    cam_params_fs["mono_cam_num"] >> _mono_cam_num;
    cam_params_fs["mono_cam_close_resolution_x"] >> mono_cam_val.mono_cam_close_resolution_x;
    cam_params_fs["mono_cam_close_resolution_y"] >> mono_cam_val.mono_cam_close_resolution_y;
    cam_params_fs["mono_cam_far_resolution_x"] >> mono_cam_val.mono_cam_far_resolution_x;
    cam_params_fs["mono_cam_far_resolution_y"] >> mono_cam_val.mono_cam_far_resolution_y;
    if(_mono_cam_num == 1)
    {
        cam_params_fs["mono_cam_matrix_close_1"] >> mono_cam_val.mono_cam_matrix_close;
        cam_params_fs["mono_cam_distcoeffs_close_1"] >> mono_cam_val.mono_cam_distcoeffs_close;
        cam_params_fs["mono_cam_matrix_far_1"] >> mono_cam_val.mono_cam_matrix_far;
        cam_params_fs["mono_cam_distcoeffs_far_1"] >> mono_cam_val.mono_cam_distcoeffs_far;
    }
    else if(_mono_cam_num == 2)
    {
        cam_params_fs["mono_cam_matrix_close_2"] >> mono_cam_val.mono_cam_matrix_close;
        cam_params_fs["mono_cam_distcoeffs_close_2"] >> mono_cam_val.mono_cam_distcoeffs_close;
        cam_params_fs["mono_cam_matrix_far_2"] >> mono_cam_val.mono_cam_matrix_far;
        cam_params_fs["mono_cam_distcoeffs_far_2"] >> mono_cam_val.mono_cam_distcoeffs_far;
    }
    else if(_mono_cam_num == 3)
    {
        cam_params_fs["mono_cam_matrix_close_3"] >> mono_cam_val.mono_cam_matrix_close;
        cam_params_fs["mono_cam_distcoeffs_close_3"] >> mono_cam_val.mono_cam_distcoeffs_close;
        cam_params_fs["mono_cam_matrix_far_3"] >> mono_cam_val.mono_cam_matrix_far;
        cam_params_fs["mono_cam_distcoeffs_far_3"] >> mono_cam_val.mono_cam_distcoeffs_far;
    }
    else if(_mono_cam_num == 4)
    {
        cam_params_fs["mono_cam_matrix_close_4"] >> mono_cam_val.mono_cam_matrix_close;
        cam_params_fs["mono_cam_distcoeffs_close_4"] >> mono_cam_val.mono_cam_distcoeffs_close;
        cam_params_fs["mono_cam_matrix_far_4"] >> mono_cam_val.mono_cam_matrix_far;
        cam_params_fs["mono_cam_distcoeffs_far_4"] >> mono_cam_val.mono_cam_distcoeffs_far;
    }
    cam_params_fs["mono_cam_close_FPS"] >> mono_cam_val.mono_cam_close_FPS;
    cam_params_fs["mono_cam_far_FPS"] >> mono_cam_val.mono_cam_far_FPS;
    cam_params_fs["mono_cam_exp_val_l"] >> mono_cam_val.mono_cam_exp_val_l;
    cam_params_fs["mono_cam_exp_val_h"] >> mono_cam_val.mono_cam_exp_val_h;
    cam_params_fs.release();
}
// Read parameters from paramsters.xml.
void Params::proc_params_xml_read(void)
{
    FileStorage proc_params_fs(proc_params_xml_dir, FileStorage::READ);
    proc_params_fs["red_h_l1"]    >> BR_HSV_range.red_h_1[0];
    proc_params_fs["red_h_h1"]    >> BR_HSV_range.red_h_1[1];
    proc_params_fs["red_s_l1"]    >> BR_HSV_range.red_s_1[0];
    proc_params_fs["red_s_h1"]    >> BR_HSV_range.red_s_1[1];
    proc_params_fs["red_v_l1"]    >> BR_HSV_range.red_v_1[0];
    proc_params_fs["red_v_h1"]    >> BR_HSV_range.red_v_1[1];
    proc_params_fs["red_h_l2"]    >> BR_HSV_range.red_h_2[0];
    proc_params_fs["red_h_h2"]    >> BR_HSV_range.red_h_2[1];
    proc_params_fs["red_s_l2"]    >> BR_HSV_range.red_s_2[0];
    proc_params_fs["red_s_h2"]    >> BR_HSV_range.red_s_2[1];
    proc_params_fs["red_v_l2"]    >> BR_HSV_range.red_v_2[0];
    proc_params_fs["red_v_h2"]    >> BR_HSV_range.red_v_2[1];
    proc_params_fs["blue_h_l1"]   >> BR_HSV_range.blue_h_1[0];
    proc_params_fs["blue_h_h1"]   >> BR_HSV_range.blue_h_1[1];
    proc_params_fs["blue_s_l1"]   >> BR_HSV_range.blue_s_1[0];
    proc_params_fs["blue_s_h1"]   >> BR_HSV_range.blue_s_1[1];
    proc_params_fs["blue_v_l1"]   >> BR_HSV_range.blue_v_1[0];
    proc_params_fs["blue_v_h1"]   >> BR_HSV_range.blue_v_1[1];
    proc_params_fs["gauss_blur_coresize"] >> armor_mono_proc_val.gauss_blur_coresize;
    proc_params_fs["oc_oprt_coresize"] >> armor_mono_proc_val.oc_oprt_coresize;
    proc_params_fs["lightbar_length_min"] >> armor_mono_proc_val.lightbar_length_min;
    proc_params_fs["lightbar_length_ratio"] >> armor_mono_proc_val.lightbar_length_ratio;
    proc_params_fs["lightbar_angle_min"] >> armor_mono_proc_val.lightbar_angle_min;
    proc_params_fs["armor_angle_diff_min"] >> armor_mono_proc_val.armor_angle_diff_min;
    proc_params_fs["armor_length_retio_min"] >> armor_mono_proc_val.armor_length_retio_min;
    proc_params_fs["armor_length_mid_y_diff"] >> armor_mono_proc_val.armor_length_mid_y_diff;
    proc_params_fs["armor_length_mid_x_diff"] >> armor_mono_proc_val.armor_length_mid_x_diff;
    proc_params_fs["cut_img_rate"] >> armor_mono_proc_val.cut_img_rate;
    proc_params_fs.release();
}


void Params::param_read(void)
{
    cam_params_xml_read();
    proc_params_xml_read();
}
