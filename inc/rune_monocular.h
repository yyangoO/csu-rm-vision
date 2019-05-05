/*
 * @ File name: rune_monocular.h
 * @ Effect: Process images to hit the rune.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef RUNE_MONOCULAR_H
#define RUNE_MONOCULAR_H


#include <vector>

#include "inc/includes.h"
#include "inc/parameters.h"


class RuneMono {
public:
    void rune_mono_proc(cv::Mat &in_img, Params params);
private:
    void hsv_proc(cv::Mat &in_img, cv::Mat &out_img, Params params);
public:
    struct FrameInfo_t {
        int frame_reso_x, frame_reso_y;
        float frame_mid_x, frame_mid_y;
    };
    FrameInfo_t frame_info;
    struct TargetInfo_t {
        float X_offset;
        float Y_offset;
        bool target_flag;
    };
    TargetInfo_t target_info;
private:
    cv::Mat fan_oc_element;
    cv::Mat target_oc_element;
    cv::RotatedRect fan_roi;
    cv::RotatedRect target_roi;
    cv::Point2f fan_rect_p[4];
    cv::Point2f target_rect_p[4];
    float fan_area, contours_area, target_area;
};

#endif // RUNE_MONOCULAR_H
