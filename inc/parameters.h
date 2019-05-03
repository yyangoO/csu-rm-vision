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
    void param_read(void);
private:
    void cam_params_xml_read(void);
    void proc_params_xml_read(void);
public:
    // Monocular camera:
    struct MonoCamVal_t
    {
        int mono_cam_close_resolution_x, mono_cam_close_resolution_y;
        float mono_cam_close_middle_x, mono_cam_close_middle_y;
        int mono_cam_close_FPS;
        int mono_cam_far_resolution_x, mono_cam_far_resolution_y;
        float mono_cam_far_middle_x, mono_cam_far_middle_y;
        int mono_cam_far_FPS;
        int mono_cam_exp_val_l, mono_cam_exp_val_h;
        cv::Mat mono_cam_matrix_close, mono_cam_distcoeffs_close;
        cv::Mat mono_cam_matrix_far, mono_cam_distcoeffs_far;
    };
    MonoCamVal_t mono_cam_val;
    // Armor:
    struct BRHSVRange_t {
        int red_h_1[2];     // Red HSV: H range 1.
        int red_s_1[2];     // Red HSV: S range 1.
        int red_v_1[2];     // Red HSV: V range 1.
        int red_h_2[2];     // Red HSV: H range 2.
        int red_s_2[2];     // Red HSV: S range 2.
        int red_v_2[2];     // Red HSV: V range 2.
        int blue_h_1[2];    // Blue HSV: H range 1.
        int blue_s_1[2];    // Blue HSV: S range 1.
        int blue_v_1[2];    // Blue HSV: V range 1.
    };
    BRHSVRange_t BR_HSV_range;
    struct ArmorMonoProcVal_t {
        int gauss_blur_coresize;        // GaussianBlur core size;
        int oc_oprt_coresize;           // Open and close operating core size;
        float lightbar_length_min;      // Based on this to filter lightbars.
        float lightbar_length_ratio;    // Based on this to filter lightbars.
        float lightbar_angle_min;       // Based on this to filter lightbars.
        float armor_angle_diff_min;     // Based on this to match lightbars.
        float armor_length_retio_min;   // Based on this to match lightbars.
        float armor_length_mid_y_diff;  // Based on this to match lightbars.
        float armor_length_mid_x_diff;  // Based on this to match lightbars.
        float armor_retio_simita;
        int cut_img_rate;
    };
    ArmorMonoProcVal_t armor_mono_proc_val;
    struct RuneMonoProcVal_t {
        int gauss_blur_coresize;
        int fan_oc_element_coresize;
        int target_oc_element_coresize;
        float actived_fan_pixel_l;
        float actived_fan_pixel_h;
        float activing_fan_pixel_l;
        float activing_fan_pixel_h;
        float target_fan_pixel_l;
        float target_fan_pixel_h;
    };
    RuneMonoProcVal_t rune_mono_proc_val;
    struct Robo_CMD_t {
        bool init_flag;
        bool set_flag;
        bool debug_flag;
        bool enemy_color;
        bool aim_or_rune;
        bool armor_type;
    };
    Robo_CMD_t robo_cmd;
private:
    int _mono_cam_num;   // Camera number.
    const std::string proc_params_xml_dir = "../csu_rm_vision_v2.1/data/proc_params.xml";
    const std::string cam_params_xml_dir = "../csu_rm_vision_v2.1/data/cam_params.xml";
};


extern Params params;


#endif // PARAMETERS_H
