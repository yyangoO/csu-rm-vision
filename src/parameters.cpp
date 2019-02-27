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
    fs.release();
//    FileStorage fs(cam_params_xml_dir, FileStorage::READ);
//    fs[""]
}


void Params::param_init(void)
{
    params_xml_read();
}
