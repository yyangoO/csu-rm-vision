/*
 * @ File name: armor_monocular.cpp
 * @ Effect: Process the low exposure value image.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "inc/includes.h"
#include "inc/armor_monocular.h"
#include "inc/parameters.h"


using namespace std;
using namespace cv;


ArmorMono armor_mono;


// Based on the last time centers to cut the range of image.
// Input image: RGB; Output image: RGB.
void ArmorMono::range_cut(Mat &in_img, Mat &out_img)
{
    Mat org_img = in_img;
}

// Used to pick up red and blue from low expusore image.
// Input image: RGB; Output image: Bin.
void ArmorMono::hsv_proc(Mat &in_img, Mat &out_img, Params params)
{
    Mat org_img = in_img;
    Mat hsv_img;
    Mat filter_img_1, filter_img_2;

    cvtColor(in_img, hsv_img, COLOR_BGR2HSV);
    filter_img_1 = Mat(hsv_img.rows, hsv_img.cols, CV_8UC3, Scalar(255, 255, 255));
    filter_img_2 = Mat(hsv_img.rows, hsv_img.cols, CV_8UC3, Scalar(255, 255, 255));
    if(params.enemy_color == ENEMY_RED)
    {
        inRange(hsv_img, Scalar(params.red_h_l1, params.red_s_l1, params.red_v_l1), \
                Scalar(params.red_h_h1, params.red_s_h1, params.red_v_h1), filter_img_1);
        inRange(hsv_img, Scalar(params.red_h_l2, params.red_s_l2, params.red_v_l2), \
                Scalar(params.red_h_h2, params.red_s_h2, params.red_v_h2), filter_img_2);
        out_img = filter_img_1 | filter_img_2;
    }
    else
    {
        inRange(hsv_img, Scalar(params.blue_h_l1, params.blue_s_l1, params.blue_v_l1), \
                Scalar(params.blue_h_h1, params.blue_s_h1, params.blue_v_h1), filter_img_1);
        out_img = filter_img_1;
    }
}

// Arm to fill the lightbar's hole.
// Input image: Bin; Output image: Bin.
void ArmorMono::fill_hole(Mat &in_img, Mat &out_img)
{
    Mat org_img = in_img;
    Mat cut_img;
    Size size = org_img.size();
    Mat temp = Mat::zeros(size.height + 2, size.width + 2, org_img.type()); // Make image bigger.
    org_img.copyTo(temp(Range(1, size.height + 1), Range(1, size.width + 1)));
    floodFill(temp, Point(0, 0), Scalar(255));
    temp(Range(1, size.height + 1), Range(1, size.width + 1)).copyTo(cut_img);
    out_img = org_img | (~cut_img);
}

// Find the lightbars, input mat must be binocolor.
// Input image: Bin; Output image: RGB.
void ArmorMono::find_lightbar(Mat &in_img, Params params)
{
    Mat org_img = in_img;
    LightBar_t lightbar_temp = {};
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    Point2f rect_p[4], rect_mid_p[4], final_mid_p[2];
    Point2f lightbar_mid_p;
    float slope = 0;
    float length[2], mid_length[2], length_ratio = 0;

    findContours(org_img, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(size_t contours_idx = 0; contours_idx < contours.size(); contours_idx++)
    {
        RotatedRect rect = minAreaRect(contours[contours_idx]); // Get the most small rect.
        rect.points(rect_p);
        length[0] = powf((rect_p[0].x - rect_p[1].x), 2) + powf((rect_p[0].y - rect_p[1].y), 2);
        length[0] = sqrtf(length[0]);
        length[1] = powf((rect_p[1].x - rect_p[2].x), 2) + powf((rect_p[1].y - rect_p[2].y), 2);
        length[1] = sqrtf(length[1]);
        // Based on the length to filter.
        if((length[0] > params.lightbar_length_min) || (length[1] > params.lightbar_length_min))
        {
            // Then based on the (height/width) to filter.
            if(length[0] > length[1])
            {
                length_ratio = length[0] / (length[1] + 0.000001);
            }
            else
            {
                length_ratio = length[1] / (length[0] + 0.000001);
            }
            if(length_ratio > params.lightbar_length_ratio)
            {
                for(size_t rect_mid_p_idx; rect_mid_p_idx < 4; rect_mid_p_idx++)
                {
                    rect_mid_p[rect_mid_p_idx].x = (rect_p[rect_mid_p_idx].x + \
                                                    rect_p[(rect_mid_p_idx + 1) % 4].x) / 2;
                    rect_mid_p[rect_mid_p_idx].y = (rect_p[rect_mid_p_idx].y + \
                                                    rect_p[(rect_mid_p_idx + 1) % 4].y) / 2;
                }
                mid_length[0] = powf(rect_mid_p[0].x + rect_mid_p[2].x, 2) + \
                                powf(rect_mid_p[0].y + rect_mid_p[2].y, 2);
                mid_length[0] = sqrtf(mid_length[0]);
                mid_length[1] = powf(rect_mid_p[1].x + rect_mid_p[3].x, 2) + \
                                powf(rect_mid_p[1].y + rect_mid_p[3].y, 2);
                mid_length[1] = sqrtf(mid_length[1]);
                if(mid_length[0] > mid_length[1])
                {
                    final_mid_p[0] = rect_mid_p[0];
                    final_mid_p[1] = rect_mid_p[2];
                }
                else
                {
                    final_mid_p[0] = rect_mid_p[1];
                    final_mid_p[1] = rect_mid_p[3];
                }
                // Based on the slope of lightbar to filter.
                slope = (float)((final_mid_p[0].y - final_mid_p[1].y) / \
                                (final_mid_p[0].x - final_mid_p[1].x + 0.000001));
                if((slope > params.lightbar_slope_min) || (slope < -params.lightbar_slope_min))
                {
                    // Finally we get the almost right lightbar's information.
                    lightbar_mid_p.x = (final_mid_p[0].x + final_mid_p[1].x) / 2;
                    lightbar_mid_p.y = (final_mid_p[0].y + final_mid_p[1].y) / 2;
                    lightbar_temp.p[0] = (Point)final_mid_p[0];
                    lightbar_temp.p[1] = (Point)final_mid_p[1];
                    lightbar_temp.mid_p = (Point)lightbar_mid_p;
                    lightbar_temp.slope = slope;
                    if(lightbar_temp.slope < 0)
                    {
                        lightbar_temp.apex_p[0].x = abs(abs(final_mid_p[0].x - \
                                                            final_mid_p[1].x) - \
                                                            lightbar_mid_p.x);
                        lightbar_temp.apex_p[1].y = abs(abs(final_mid_p[0].y - \
                                                            final_mid_p[1].y) - \
                                                            lightbar_mid_p.y);
                        lightbar_temp.apex_p[1].x = abs(abs(final_mid_p[0].x - \
                                                            final_mid_p[1].x) + \
                                                            lightbar_mid_p.x);
                        lightbar_temp.apex_p[0].y = abs(abs(final_mid_p[0].y - \
                                                            final_mid_p[1].y) + \
                                                            lightbar_mid_p.y);
                    }
                    else
                    {
                        lightbar_temp.apex_p[0].x = abs(abs(final_mid_p[0].x - \
                                                            final_mid_p[1].x) - \
                                                            lightbar_mid_p.x);
                        lightbar_temp.apex_p[0].y = abs(abs(final_mid_p[0].y - \
                                                            final_mid_p[1].y) - \
                                                            lightbar_mid_p.y);
                        lightbar_temp.apex_p[1].x = abs(abs(final_mid_p[0].x - \
                                                            final_mid_p[1].x) + \
                                                            lightbar_mid_p.x);
                        lightbar_temp.apex_p[1].y = abs(abs(final_mid_p[0].y - \
                                                            final_mid_p[1].y) + \
                                                            lightbar_mid_p.y);
                    }
                    light_bars.push_back(lightbar_temp);
                }
            }
        }
    }
}


// Find Armors.
// Input image: Bin(coutours); Output image: RGB.
void ArmorMono::find_armor(Params params)
{
//    ArmorROI_t armor_roi_temp = {};
//    for(int lb_idx = 0; lb_idx < light_bars.size(); lb_idx++)
//    {
//        for(int lb_idx_idx = lb_idx + 1; lb_idx_idx < light_bars.size(); lb_idx_idx++)
//        // First based on the angle of lightbars' different to match them.
//        if(abs(light_bars.at(lb_idx) - light_bars.at(lb_idx_idx)) < params.armor_slope_diff_min)
//        {
//            // Then based on the length different to match them. (length > 0)
//            if((light_bars.at(lb_idx).length / light_bars.at(lb_idx_idx))    \
//                                    < (1 + params.armor_length_diff_min)  || \
//                (light_bars.at(lb_idx).length / lightbars.at(lb_idx_idx))    \
//                                    > (1 - params.armor_length_diff_min))
//            {
                
//            }
//        }
//    }
}

// Clear vectors.
void ArmorMono::vector_clear(void)
{
    light_bars.clear();
    armors.clear();
}

// Monocular camera proc.
void ArmorMono::armor_mono_proc(Mat &in_img, Params params)
{
    vector_clear();
    hsv_proc(in_img, in_img, params);
    fill_hole(in_img, in_img);
    GaussianBlur(in_img, in_img, Size(params.gauss_blur_coresize, params.gauss_blur_coresize), 0);
    oc_element = getStructuringElement(MORPH_RECT, Size(params.oc_oprt_coresize, \
                                                        params.oc_oprt_coresize));
    erode(in_img, in_img, oc_element);
    dilate(in_img, in_img, oc_element);
    find_lightbar(in_img, params);
}
