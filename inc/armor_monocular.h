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
    void range_cut(cv::Mat &in_img, cv::Mat &out_img);
    void hsv_proc(cv::Mat &in_img, cv::Mat &out_img, Params params);
    void find_lightbar(cv::Mat &in_img, Params params);
    void find_armor(Params params);
    void target_detect(Params params);
    void angle_slove(Params params);
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
        float X_offset;
        float Y_offset;
        float Z_offset;
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

    bool roi_overflow_flag;       // Flag of lightbar's crossing.
    cv::Mat oc_element;
    cv::Point last_center;
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
        float trust_level;
    };
    std::vector<ArmorROI_t> armors;
};


extern ArmorMono armor_mono;


#endif // ARMOR_MONOCULAR_H
