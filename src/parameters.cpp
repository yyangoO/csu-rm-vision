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
void Params::params_xml_read(void)
{
    FileStorage cam_fs(cam_params_xml_dir, FileStorage::READ);
    cam_fs["mono_cam_matrix_640_1"] >> mono_cam_matrix;
    cam_fs["mono_cam_distcoeffs_640_1"] >> mono_cam_distcoeffs;
    cam_fs.release();

    FileStorage fs(proc_params_xml_dir, FileStorage::READ);
    fs["enemy_color"] >> enemy_color;
    fs["red_h_l1"]    >> red_h_l1;
    fs["red_h_h1"]    >> red_h_h1;
    fs["red_s_l1"]    >> red_s_l1;
    fs["red_s_h1"]    >> red_s_h1;
    fs["red_v_l1"]    >> red_v_l1;
    fs["red_v_h1"]    >> red_v_h1;
    fs["red_h_l2"]    >> red_h_l2;
    fs["red_h_h2"]    >> red_h_h2;
    fs["red_s_l2"]    >> red_s_l2;
    fs["red_s_h2"]    >> red_s_h2;
    fs["red_v_l2"]    >> red_v_l2;
    fs["red_v_h2"]    >> red_v_h2;
    fs["blue_h_l1"]   >> blue_h_l1;
    fs["blue_h_h1"]   >> blue_h_h1;
    fs["blue_s_l1"]   >> blue_s_l1;
    fs["blue_s_h1"]   >> blue_s_h1;
    fs["blue_v_l1"]   >> blue_v_l1;
    fs["blue_v_h1"]   >> blue_v_h1;
    fs["gauss_blur_coresize"] >> gauss_blur_coresize;
    fs["oc_oprt_coresize"] >> oc_oprt_coresize;
    fs["lightbar_length_min"] >> lightbar_length_min;
    fs["lightbar_length_retio"] >> lightbar_length_ratio;
    fs["lightbar_angle_min"] >> lightbar_angle_min;
    fs["armor_angle_diff_min"] >> armor_angle_diff_min;
    fs["armor_length_retio_min"] >> armor_length_retio_min;
    fs["armor_length_mid_y_diff"] >> armor_length_mid_y_diff;
    fs["armor_length_mid_x_diff"] >> armor_length_mid_x_diff;
    fs["cut_img_rate"] >> cut_img_rate;
    fs.release();
}


void Params::param_init(void)
{
    params_xml_read();
}
