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


class ArmorMono
{
public:
    void armor_mono_proc(cv::Mat &in_img, Params params);
private:
    void range_cut(cv::Mat &in_img, cv::Mat &out_img, Params params);
    void hsv_proc(cv::Mat &in_img, cv::Mat &out_img, Params params);
    void find_lightbar(cv::Mat &in_img, Params params);
    void find_armor(Params params);
    void target_detect(const cv::Mat cam_img, Params params);
    float distance_slove(cv::Point2f *apex_point, Params params);
    // vector manage:
    void vector_clear(void);
public:
    // Debug imgs:
    cv::Mat hsv_img;
    cv::Mat grag_img;
    cv::Mat bin_img;
    cv::Mat mono_img;
    // order informations:
    struct OrderInfo {
        // General order:
        bool enemy_color;   // Red: 0, Blue: 1.
        // Hero order:
        bool bullet_type;
        // Infantry order:
        bool aim_or_rune;
    };
    OrderInfo order_info;
    // Final result:
    struct TargetInfo {
        // General info:
        int16_t X_offset;     // -32767 - 32767.
        int16_t Y_offset;     // -32767 - 32767.
        uint8_t Z_offset;     // 0 - 255: 0 - 8m
        // Sentry info:
        bool shut_flag;
        char enemy_type;
    };
    TargetInfo target_info;
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
    cv::Point2f last_center = {MONO_IMAGE_CENTER_X, MONO_IMAGE_CENTER_Y};
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
    bool target_flag;
};


extern ArmorMono armor_mono;


#endif // ARMOR_MONOCULAR_H
