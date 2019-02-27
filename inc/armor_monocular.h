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
    void fill_hole(cv::Mat &in_img, cv::Mat &out_img);
    void find_lightbar(cv::Mat &in_img, Params params);
    void find_armor(Params params);
    // vector manage:
    void vector_clear(void);
public:
    // Debug imgs:
    cv::Mat hsv_img;
    cv::Mat grag_img;
    cv::Mat bin_img;
    cv::Mat mono_img;
    // Final result:
    struct ArmorROI_t {
        cv::Point apex_point[4];
        cv::Point light_point[4];
        cv::Point center;
        float trust_level;
    };
    std::vector<ArmorROI_t> armors;
private:
    cv::Point last_center;
    struct LightBar_t {
        cv::Point p[2];         // Light bar, describe as two points.
        cv::Point apex_p[2];    // Armor upright line, describe as two points.
        cv::Point mid_p;        // Light bar's middle point.
        float angle;
        float length;           // Pixel unit.
    };
    std::vector<LightBar_t> light_bars;
    cv::Mat oc_element;
};


extern ArmorMono armor_mono;


#endif // ARMOR_MONOCULAR_H
