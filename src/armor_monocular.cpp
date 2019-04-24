/*
 * @ File name: armor_monocular.cpp
 * @ Effect: Process the low exposure value image to detect armor.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "iostream"
#include "iomanip"
#include "math.h"
#include "sys/time.h"
#include "inc/includes.h"
#include "inc/armor_monocular.h"
#include "inc/parameters.h"


using namespace std;
using namespace cv;


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
    if(params.robo_cmd.enemy_color == RIN_ENEMY_RED)
    {
        inRange(hsv_img, \
                Scalar(params.BR_HSV_range.red_h_1[0], params.BR_HSV_range.red_s_1[0], params.BR_HSV_range.red_s_1[0]), \
                Scalar(params.BR_HSV_range.red_h_1[1], params.BR_HSV_range.red_s_1[1], params.BR_HSV_range.red_s_1[1]), \
                filter_img_1);
        inRange(hsv_img, \
                Scalar(params.BR_HSV_range.red_h_2[0], params.BR_HSV_range.red_s_2[0], params.BR_HSV_range.red_s_2[0]), \
                Scalar(params.BR_HSV_range.red_h_2[1], params.BR_HSV_range.red_s_2[1], params.BR_HSV_range.red_s_2[1]), \
                filter_img_2);
        out_img = filter_img_1 | filter_img_2;
    }
    else if(params.robo_cmd.enemy_color == RIN_ENEMY_BLUE)
    {
        inRange(hsv_img, \
                Scalar(params.BR_HSV_range.blue_h_1[0], params.BR_HSV_range.blue_s_1[0], params.BR_HSV_range.blue_v_1[0]), \
                Scalar(params.BR_HSV_range.blue_h_1[1], params.BR_HSV_range.blue_s_1[1], params.BR_HSV_range.blue_v_1[1]), \
                filter_img_1);
        out_img = filter_img_1;
    }
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
    Point2f apex_remap_temp, lightbar_remap_temp;
    RotatedRect rotated_rect;
    float slope = 0, angle = 0;
    float length[2], mid_length[2], length_ratio = 0;

    findContours(org_img, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(size_t contours_idx = 0; contours_idx < contours.size(); contours_idx++)
    {
        rotated_rect = minAreaRect(contours[contours_idx]); // Get the most small rect.
        rotated_rect.points(rect_p);
        length[0] = powf((rect_p[0].x - rect_p[1].x), 2) + powf((rect_p[0].y - rect_p[1].y), 2);
        length[0] = sqrtf(length[0]);
        length[1] = powf((rect_p[1].x - rect_p[2].x), 2) + powf((rect_p[1].y - rect_p[2].y), 2);
        length[1] = sqrtf(length[1]);

        // Based on the length to filter.
        if((length[0] > params.armor_mono_proc_val.lightbar_length_min) || \
           (length[1] > params.armor_mono_proc_val.lightbar_length_min))
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
            if(length_ratio > params.armor_mono_proc_val.lightbar_length_ratio)
            {
                for(size_t rect_mid_p_idx = 0; rect_mid_p_idx < 4; rect_mid_p_idx++)
                {
                    rect_mid_p[rect_mid_p_idx].x = (rect_p[rect_mid_p_idx].x + \
                                                    rect_p[(rect_mid_p_idx + 1) % 4].x) / 2;
                    rect_mid_p[rect_mid_p_idx].y = (rect_p[rect_mid_p_idx].y + \
                                                    rect_p[(rect_mid_p_idx + 1) % 4].y) / 2;
                }
                mid_length[0] = powf((rect_mid_p[0].x - rect_mid_p[2].x), 2) + \
                                powf((rect_mid_p[0].y - rect_mid_p[2].y), 2);
                mid_length[0] = sqrtf(mid_length[0]);
                mid_length[1] = powf((rect_mid_p[1].x - rect_mid_p[3].x), 2) + \
                                powf((rect_mid_p[1].y - rect_mid_p[3].y), 2);
                mid_length[1] = sqrtf(mid_length[1]);
                if(mid_length[0] > mid_length[1])
                {
                    final_mid_p[0] = rect_mid_p[0];
                    final_mid_p[1] = rect_mid_p[2];
                    if(final_mid_p[0].y > final_mid_p[1].y)
                    {
                        lightbar_remap_temp = final_mid_p[0];
                        final_mid_p[0] = final_mid_p[1];
                        final_mid_p[1] = lightbar_remap_temp;
                    }
                }
                else
                {
                    final_mid_p[0] = rect_mid_p[1];
                    final_mid_p[1] = rect_mid_p[3];
                    if(final_mid_p[0].y > final_mid_p[1].y)
                    {
                        lightbar_remap_temp = final_mid_p[0];
                        final_mid_p[0] = final_mid_p[1];
                        final_mid_p[1] = lightbar_remap_temp;
                    }
                }
                // Based on the slope of lightbar to filter.
                slope = (float)((final_mid_p[0].y - final_mid_p[1].y) / \
                                (final_mid_p[0].x - final_mid_p[1].x + 0.000001));
                angle = atan(slope);
                if(abs(angle) > params.armor_mono_proc_val.lightbar_angle_min)
                {
                    // Finally we get the almost right lightbar's information.
                    lightbar_mid_p.x = (final_mid_p[0].x + final_mid_p[1].x) / 2;
                    lightbar_mid_p.y = (final_mid_p[0].y + final_mid_p[1].y) / 2;
                    lightbar_temp.p[0] = (Point)final_mid_p[0];
                    lightbar_temp.p[1] = (Point)final_mid_p[1];
                    lightbar_temp.mid_p = (Point)lightbar_mid_p;
                    lightbar_temp.angle = angle;
                    // Get length
                    lightbar_temp.length = powf((lightbar_temp.p[0].x - lightbar_temp.p[1].x), 2) + \
                                           powf((lightbar_temp.p[0].y - lightbar_temp.p[1].y), 2);
                    lightbar_temp.length = sqrtf(lightbar_temp.length);
                    if(lightbar_temp.angle < 0)
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
                        // Based on Y value to remap
                        if(lightbar_temp.apex_p[0].y > lightbar_temp.apex_p[1].y)
                        {
                            apex_remap_temp = lightbar_temp.apex_p[0];
                            lightbar_temp.apex_p[0] = lightbar_temp.apex_p[1];
                            lightbar_temp.apex_p[1] = apex_remap_temp;
                        }
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
                        // Based on Y value to remap
                        if(lightbar_temp.apex_p[0].y > lightbar_temp.apex_p[1].y)
                        {
                            apex_remap_temp = lightbar_temp.apex_p[0];
                            lightbar_temp.apex_p[0] = lightbar_temp.apex_p[1];
                            lightbar_temp.apex_p[1] = apex_remap_temp;
                        }
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
    ArmorROI_t armor_roi_temp = {};
    for(size_t lb_idx = 0; lb_idx < light_bars.size(); lb_idx++)
    {
        for(size_t lb_idx_idx = lb_idx + 1; lb_idx_idx < light_bars.size(); lb_idx_idx++)
        {
            if((light_bars.at(lb_idx).angle > params.armor_mono_proc_val.armor_retio_simita) && (light_bars.at(lb_idx).angle < 1.5708))
                light_bars.at(lb_idx).angle = 1.5708;
            if((light_bars.at(lb_idx).angle > -1.5708) && (light_bars.at(lb_idx).angle < -params.armor_mono_proc_val.armor_retio_simita))
                light_bars.at(lb_idx).angle = 1.5708;
            if((light_bars.at(lb_idx_idx).angle > params.armor_mono_proc_val.armor_retio_simita) && (light_bars.at(lb_idx_idx).angle < 1.5708))
                light_bars.at(lb_idx_idx).angle = 1.5708;
            if((light_bars.at(lb_idx_idx).angle > -1.5708) && (light_bars.at(lb_idx_idx).angle < params.armor_mono_proc_val.armor_retio_simita))
                light_bars.at(lb_idx_idx).angle = 1.5708;
            // First based on the angle of lightbars' different to match them.
            if(abs(light_bars.at(lb_idx).angle - light_bars.at(lb_idx_idx).angle) < \
               params.armor_mono_proc_val.armor_angle_diff_min)
            {
                // Then based on the length different to match them. (length > 0)
                if((light_bars.at(lb_idx).length / (light_bars.at(lb_idx_idx).length + 0.000001))    \
                                        < (1 + params.armor_mono_proc_val.armor_length_retio_min) || \
                   (light_bars.at(lb_idx).length / (light_bars.at(lb_idx_idx).length + 0.000001))    \
                                        > (1 - params.armor_mono_proc_val.armor_length_retio_min))
                {
                    // Based on the middle point'y to match.
                    if(abs(light_bars.at(lb_idx).mid_p.y - light_bars.at(lb_idx_idx).mid_p.y) <    \
                           (params.armor_mono_proc_val.armor_length_mid_y_diff *                   \
                            ((light_bars.at(lb_idx).length + light_bars.at(lb_idx_idx).length) / 2 \
                             )))
                    {
                        // Based on the middle point'x to match
                        if(abs(light_bars.at(lb_idx).mid_p.x - light_bars.at(lb_idx_idx).mid_p.x) <     \
                                (params.armor_mono_proc_val.armor_length_mid_x_diff *                   \
                                 ((light_bars.at(lb_idx).length + light_bars.at(lb_idx_idx).length) / 2 \
                                  )))
                        {
                            // Finally we have these armors.
                            armor_roi_temp.apex_point[0] = light_bars.at(lb_idx).apex_p[0];
                            armor_roi_temp.apex_point[1] = light_bars.at(lb_idx).apex_p[1];
                            armor_roi_temp.apex_point[2] = light_bars.at(lb_idx_idx).apex_p[1];
                            armor_roi_temp.apex_point[3] = light_bars.at(lb_idx_idx).apex_p[0];
                            armor_roi_temp.light_point[0] = light_bars.at(lb_idx).p[0];
                            armor_roi_temp.light_point[1] = light_bars.at(lb_idx).p[1];
                            armor_roi_temp.light_point[2] = light_bars.at(lb_idx_idx).p[1];
                            armor_roi_temp.light_point[3] = light_bars.at(lb_idx_idx).p[0];
                            armor_roi_temp.center.x = (light_bars.at(lb_idx).mid_p.x + \
                                                       light_bars.at(lb_idx_idx).mid_p.x) / 2;
                            armor_roi_temp.center.y = (light_bars.at(lb_idx).mid_p.y + \
                                                       light_bars.at(lb_idx_idx).mid_p.y) / 2;
                            armor_roi_temp.distance = distance_slove(armor_roi_temp.light_point, params);
                            armor_roi_temp.verical_length = (light_bars.at(lb_idx).length + \
                                                             light_bars.at(lb_idx_idx).length) / 2;
                            armor_roi_temp.flat_angle = atan((light_bars.at(lb_idx).mid_p.y - light_bars.at(lb_idx_idx).mid_p.y) / \
                                                             (light_bars.at(lb_idx).mid_p.x - light_bars.at(lb_idx_idx).mid_p.x + \
                                                              0.000001));
                            armors.push_back(armor_roi_temp);
                        }
                    }
                }
            }
        }
    }
}

float ArmorMono::distance_slove(Point2f *apex_point, Params params)
{
    Mat rvec, tvec;
    float distance;
    vector<Point2f> real_point;

        real_point.clear();
        for(size_t apex_point_idx = 0; apex_point_idx < 4; apex_point_idx++)
        {
            real_point.push_back(*(apex_point + apex_point_idx));
        }
        solvePnP(small_armor_world_point, real_point, params.mono_cam_val.mono_cam_matrix_far, \
                 params.mono_cam_val.mono_cam_distcoeffs_far, rvec, tvec, false, CV_ITERATIVE);
        distance = (float)sqrt(tvec.at<double>(0, 0) * tvec.at<double>(0, 0) + \
                               tvec.at<double>(1, 0) * tvec.at<double>(1, 0) + \
                               tvec.at<double>(2, 0) * tvec.at<double>(2, 0));
        return distance;
}

// Detect target.
void ArmorMono::target_detect(void)
{
    float hightset_trust_lev = 0, curr_trust_lev = 0;
    target_flag = false;
    final_target = {};
    for(size_t armor_idx = 0; armor_idx < armors.size(); armor_idx++)
    {
        curr_trust_lev = 5.0 * (CV_PI - armors.at(armor_idx).flat_angle) + \
                         1.0 * armors.at(armor_idx).verical_length + \
                         0.1 * armors.at(armor_idx).distance;
        if(curr_trust_lev > hightset_trust_lev)
        {
            final_target = armors.at(armor_idx);
            target_info.X_offset = final_target.center.x - MONO_IMAGE_CENTER_X;
            target_info.Y_offset = final_target.center.y - MONO_IMAGE_CENTER_Y;
            target_info.Z_offset = final_target.distance;
            hightset_trust_lev = curr_trust_lev;
        }
        target_flag = true;
    }
    if(target_flag == false)
    {
        target_info.X_offset = 0;
        target_info.Y_offset = 0;
        target_info.Z_offset = 0;
    }
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
    struct timeval tv1, tv2;
    struct timezone tz1, tz2;
    Mat debug_cam_img;
    string debug_fps = "fps: ";
    string debug_proc_tim = "porc tim: ";
    string debug_gauss_blur_coresize = "gauss blur coresize: ";
    string debug_oc_opt_coresize = "oc opt coresize: ";
    string debug_cut_img_rate = "cut img rate: ";
    string debug_lightbar_num = "lightbar number: ";
    string debug_lightbar_length_min = "lightbar length min: ";
    string debug_lightbar_length_retio = "lightbar length retio: ";
    string debug_lightbar_angle_min = "lightbar angle min: ";
    string debug_armor_num = "armor number: ";
    string debug_lightbar_angle_diff = "lightbar angle diff: ";
    string debug_lightbar_length_diff = "lightbar length diff: ";
    string debug_lightbar_mid_y_diff = "lightbar mid_y diff: ";
    string debug_lightbar_mid_x_diff = "lightbar mid_x diff: ";
    string debug_target_distance = "target distance: ";
    string debug_target_flag = "target flag: ";
    string debug_xyz = "target XYZ: ";
    float debug_proc_tim_val;

    const Mat cam_img = in_img.clone();   // Should not be change.

    if(params.robo_cmd.debug_flag == true)
    {
        gettimeofday(&tv1, &tz1);
    }

    vector_clear();
    hsv_proc(in_img, in_img, params);
    fill_hole(in_img, in_img);
    GaussianBlur(in_img, in_img, Size(params.armor_mono_proc_val.gauss_blur_coresize, \
                                      params.armor_mono_proc_val.gauss_blur_coresize), 0);
    oc_element = getStructuringElement(MORPH_RECT, \
                                       Size(params.armor_mono_proc_val.oc_oprt_coresize, \
                                            params.armor_mono_proc_val.oc_oprt_coresize));
    erode(in_img, in_img, oc_element);
    dilate(in_img, in_img, oc_element);

    find_lightbar(in_img, params);
    find_armor(params);
    target_detect();

    if(params.robo_cmd.debug_flag == true)
    {
        gettimeofday(&tv2, &tz2);
        debug_proc_tim_val = tv2.tv_sec * 1000 + tv2.tv_usec / 1000 - \
                             tv1.tv_sec * 1000 - tv1.tv_usec / 1000;
    }

    if(params.robo_cmd.debug_flag == true)
    {
        debug_cam_img = cam_img.clone();
        // Draw proc informations.
        for(size_t lightbar_debug_idx = 0; lightbar_debug_idx < light_bars.size(); lightbar_debug_idx++)
        {
            line(debug_cam_img, light_bars.at(lightbar_debug_idx).p[0], \
                 light_bars.at(lightbar_debug_idx).p[1], Scalar(255, 0, 0), 2);
            line(debug_cam_img, light_bars.at(lightbar_debug_idx).apex_p[0], \
                 light_bars.at(lightbar_debug_idx).apex_p[1], Scalar(0, 255, 0), 1);
            circle(debug_cam_img, light_bars.at(lightbar_debug_idx).mid_p, \
                   3, Scalar(255, 255,255), -1);
        }
        for(size_t armor_idx = 0; armor_idx < armors.size(); armor_idx++)
        {
            for(size_t apex_idx = 0; apex_idx < 4; apex_idx++)
            {
                line(debug_cam_img, armors.at(armor_idx).apex_point[apex_idx], \
                     armors.at(armor_idx).apex_point[(apex_idx + 1) % 4], \
                     Scalar(0, 0, 255), 1);
            }
        }
        circle(debug_cam_img, final_target.center, 3, Scalar(255, 255,255), -1);
        // Show informations.
        debug_fps = debug_fps + to_string(params.mono_cam_val.mono_cam_close_FPS);
        putText(debug_cam_img, debug_fps, Point(0, 15), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(255, 255, 0), 1.5);
        debug_proc_tim = debug_proc_tim + to_string(debug_proc_tim_val);
        putText(debug_cam_img, debug_proc_tim, Point(0, 30), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(255, 255, 0), 1.5);
        debug_gauss_blur_coresize = debug_gauss_blur_coresize + \
                                    to_string(params.armor_mono_proc_val.gauss_blur_coresize);
        putText(debug_cam_img, debug_gauss_blur_coresize, Point(0, 45), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_oc_opt_coresize = debug_oc_opt_coresize + \
                                to_string(params.armor_mono_proc_val.oc_oprt_coresize);
        putText(debug_cam_img, debug_oc_opt_coresize, Point(0, 60), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_cut_img_rate = debug_cut_img_rate + \
                             to_string(params.armor_mono_proc_val.cut_img_rate);
        putText(debug_cam_img, debug_cut_img_rate, Point(0, 75), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_num = debug_lightbar_num + to_string(light_bars.size());
        putText(debug_cam_img, debug_lightbar_num, Point(0, 90), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 0, 255), 1.5);
        debug_lightbar_length_min = debug_lightbar_length_min +
                                    to_string(params.armor_mono_proc_val.lightbar_length_min);
        putText(debug_cam_img, debug_lightbar_length_min, Point(0, 105), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_length_retio = debug_lightbar_length_retio + \
                                      to_string(params.armor_mono_proc_val.lightbar_length_ratio);
        putText(debug_cam_img, debug_lightbar_length_retio, Point(0, 120), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_angle_min = debug_lightbar_angle_min + \
                                   to_string(params.armor_mono_proc_val.lightbar_angle_min);
        putText(debug_cam_img, debug_lightbar_angle_min, Point(0, 135), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_armor_num = debug_armor_num + to_string(armors.size());
        putText(debug_cam_img, debug_armor_num, Point(0, 150), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 0, 255), 1.5);
        debug_lightbar_angle_diff = debug_lightbar_angle_diff + \
                                    to_string(params.armor_mono_proc_val.armor_angle_diff_min);
        putText(debug_cam_img, debug_lightbar_angle_diff, Point(0, 165), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_length_diff = debug_lightbar_length_diff + \
                                     to_string(params.armor_mono_proc_val.armor_length_retio_min);
        putText(debug_cam_img, debug_lightbar_length_diff, Point(0, 180), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_mid_y_diff = debug_lightbar_mid_y_diff + \
                                    to_string(params.armor_mono_proc_val.armor_length_mid_y_diff);
        putText(debug_cam_img, debug_lightbar_mid_y_diff, Point(0, 195), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_lightbar_mid_x_diff = debug_lightbar_mid_x_diff + \
                                    to_string(params.armor_mono_proc_val.armor_length_mid_x_diff);
        putText(debug_cam_img, debug_lightbar_mid_x_diff, Point(0, 210), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_target_distance = debug_target_distance + to_string(final_target.distance);
        putText(debug_cam_img, debug_target_distance, Point(0, 225), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 0, 255), 1.5);

        if(params.robo_cmd.enemy_color == RIN_ENEMY_RED)
        {
            putText(debug_cam_img, "enemy red", Point(0, 440), \
                    DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 0, 255), 1.5);
        }
        else
        {
            putText(debug_cam_img, "enemy blue", Point(0, 440), \
                    DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);
        }
        debug_xyz = debug_xyz + to_string((int16_t)target_info.X_offset) + " " \
                              + to_string((int16_t)target_info.Y_offset) + " " \
                              + to_string((int16_t)target_info.Z_offset);
        putText(debug_cam_img, debug_xyz, Point(0, 455), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);
        debug_target_flag = debug_target_flag + to_string(target_flag);
        putText(debug_cam_img, debug_target_flag, Point(0, 470), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);

        cvNamedWindow("debug_image", CV_WINDOW_AUTOSIZE);
        imshow("debug_image", debug_cam_img);
        waitKey(1);
    }
}
