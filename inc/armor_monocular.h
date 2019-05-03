/*
 * @ File name: armor_monocular.h
 * @ Effect: Process the low exposure value image to detect armor.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef ARMOR_MONOCULAR_H
#define ARMOR_MONOCULAR_H


#include <vector>

#include "inc/includes.h"
#include "inc/parameters.h"
#include "sys/time.h"


class ArmorMono
{
public:
    void armor_mono_proc(cv::Mat &in_img, Params params);
private:
    void range_cut(cv::Mat &in_img, cv::Mat &out_img, Params params);
    void hsv_proc(cv::Mat &in_img, cv::Mat &out_img, Params params);
    void find_lightbar(cv::Mat &in_img, Params params);
    void find_armor(Params params);
    void target_detect(void);
    float distance_slove(cv::Point2f *apex_point, Params params);
    // vector manage:
    void vector_clear(void);
public:
    // Frame infomation:
    struct FrameInfo_t {
        int frame_reso_x, frame_reso_y;
        float frame_mid_x, frame_mid_y;
    };
    FrameInfo_t frame_info;
    // Final result:
    struct TargetInfo_t {
        // General info:
        float X_offset;
        float Y_offset;
        float Z_offset;
        bool target_flag;
        // Sentry info:
        bool shut_flag;
        char enemy_type;
    };
    TargetInfo_t target_info;
private:
    const std::vector<cv::Point3f> small_armor_world_point = std::vector<cv::Point3f> {
        cv::Point3f(-6.75f, -2.75f, 0),
        cv::Point3f(-6.75f, 2.75f, 0),
        cv::Point3f(6.75f, 2.75f, 0),
        cv::Point3f(6.75f, -2.75f, 0)
    };
    const std::vector<cv::Point3f> big_armor_world_point = std::vector<cv::Point3f> {
        cv::Point3f(-6.75f, -2.75f, 0),
        cv::Point3f(-6.75f, 2.75f, 0),
        cv::Point3f(6.75f, 2.75f, 0),
        cv::Point3f(6.75f, -2.75f, 0)
    };
    struct LightBar_t {
        cv::Point2f p[2];         // Light bar, describe as two points.
        cv::Point2f apex_p[2];    // Armor upright line, describe as two points.
        cv::Point2f mid_p;        // Light bar's middle point.
        float angle;
        float length;             // Pixel unit.
    };
    std::vector<LightBar_t> light_bars;
    struct ArmorROI_t {
        cv::Point2f apex_point[4];
        cv::Point2f light_point[4];
        cv::Point2f center;
        float verical_length;
        float flat_angle;
        float distance;
    };
    std::vector<ArmorROI_t> armors;
    ArmorROI_t final_target;
    bool roi_overflow_flag;       // Flag of lightbar's crossing.
    cv::Mat oc_element;
    cv::Size track_roi;
    cv::Point2f last_center;
};


#endif // ARMOR_MONOCULAR_H
