/*
 * @ File name: parameters.h
 * @ Effect: Arm to manage the system's paramsters.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/8
 */


#ifndef PARAMETERS_H
#define PARAMETERS_H


#include <string>

#include "inc/includes.h"


class Params
{
public:
    void param_init(void);
private:
    void params_xml_read(void);
public:
    // Global:
    typedef enum {
        no_debug = 0,
        debug_mono_hsv_img = 1,
        debug_mono_gray_img,
    } DebugType;
    DebugType debug_type;
    typedef enum {
        no_err = 0,
        read_params_xml_fail = 1,
    } ErrType;
    ErrType err_type;
    bool enemy_color;   // 1: Red, 0: Blue.

    // Monocular camera:
    int mono_cam_width, mono_cam_height;
    int mono_cam_exp;   // Monocular camera exposure.
    cv::Mat mono_cam_matrix;
    cv::Mat mono_cam_distcoeffs;

    // Binocular camera:
    int bino_cam_width, bino_cam_height;
    int bino_cam_exp;   // Binocular camera exposure.



    // Armor:
    int red_h_l1, red_h_h1;     // Red HSV: H range 1.
    int red_s_l1, red_s_h1;     // Red HSV: S range 1.
    int red_v_l1, red_v_h1;     // Red HSV: V range 1.
    int red_h_l2, red_h_h2;     // Red HSV: H range 2.
    int red_s_l2, red_s_h2;     // Red HSV: S range 2.
    int red_v_l2, red_v_h2;     // Red HSV: V range 2.

    int blue_h_l1, blue_h_h1;   // Blue HSV: H range 1.
    int blue_s_l1, blue_s_h1;   // Blue HSV: S range 1.
    int blue_v_l1, blue_v_h1;   // Blue HSV: V range 1.

    int gauss_blur_coresize;    // GaussianBlur core size;
    int oc_oprt_coresize;       // Open and close operating core size;

    float lightbar_length_min;    // Based on this to filter lightbars.
    float lightbar_length_ratio;  // Based on this to filter lightbars.
    float lightbar_angle_min;     // Based on this to filter lightbars.

    float armor_angle_diff_min;     // Based on this to match lightbars.
    float armor_length_retio_min;   // Based on this to match lightbars.
    float armor_length_mid_y_diff;  // Based on this to match lightbars.
    float armor_length_mid_x_diff;  // Based on this to match lightbars.

    int cut_img_rate;
private:
    const std::string proc_params_xml_dir = "../csu_rm_vision_v2.1/data/proc_params.xml";
    const std::string cam_params_xml_dir = "../csu_rm_vision_v2.1/data/cam_params.xml";
};


extern Params params;


#endif // PARAMETERS_H
