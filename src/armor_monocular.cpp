/*
 * @ File name: armor_monocular.cpp
 * @ Effect: Process the low exposure value image.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "iostream"
#include "iomanip"
#include "math.h"

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


// Find the lightbars, input mat must be binocolor.
// Input image: Bin; Output image: RGB.
void ArmorMono::find_lightbar(Mat &in_img, Params params)
{
//#define ARMOR_MONO_LIGHTBAR_DEBUG

    Mat org_img = in_img;
#ifdef ARMOR_MONO_LIGHTBAR_DEBUG
//    cvNamedWindow("debug", CV_WINDOW_AUTOSIZE);
    Mat lightbar_debug_mat = Mat(in_img.rows, in_img.cols, CV_8UC3, Scalar(0, 0, 0));
#endif
    LightBar_t lightbar_temp = {};
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    Point2f rect_p[4], rect_mid_p[4], final_mid_p[2];
    Point2f lightbar_mid_p;
    Point2f apex_remap_temp, lightbar_remap_temp;
    float slope = 0, angle = 0;
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

#ifdef ARMOR_MONO_LIGHTBAR_DEBUG
        for(size_t rect_idx = 0; rect_idx < 4; rect_idx++)
        {
            line(lightbar_debug_mat, rect_p[rect_idx], rect_p[(rect_idx + 1) % 4], \
                 Scalar(0, 255, 0), 1);
        }
#endif
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
                if(abs(angle) > params.lightbar_angle_min)
                {
                    // Finally we get the almost right lightbar's information.
                    lightbar_mid_p.x = (final_mid_p[0].x + final_mid_p[1].x) / 2;
                    lightbar_mid_p.y = (final_mid_p[0].y + final_mid_p[1].y) / 2;
                    lightbar_temp.p[0] = (Point)final_mid_p[0];
                    lightbar_temp.p[1] = (Point)final_mid_p[1];
                    lightbar_temp.mid_p = (Point)lightbar_mid_p;
                    lightbar_temp.angle = angle;
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

#ifdef ARMOR_MONO_LIGHTBAR_DEBUG
        cout << "-----------------------------------------" << endl;
        cout << "                lightbar                 " << endl;
        for(size_t lightbar_debug_idx = 0; lightbar_debug_idx < light_bars.size(); lightbar_debug_idx++)
        {
            string debug_lightbar_num = " No.";
            debug_lightbar_num = debug_lightbar_num + to_string((int)lightbar_debug_idx);

            line(lightbar_debug_mat, light_bars.at(lightbar_debug_idx).p[0], \
                 light_bars.at(lightbar_debug_idx).p[1], Scalar(255, 0, 0), 2);
            putText(lightbar_debug_mat, debug_lightbar_num, light_bars.at(lightbar_debug_idx).p[0], \
                    2, 0.5, Scalar(255, 255, 255), 1);
            line(lightbar_debug_mat, light_bars.at(lightbar_debug_idx).apex_p[0], \
                 light_bars.at(lightbar_debug_idx).apex_p[1], Scalar(0, 255, 0), 1);
            circle(lightbar_debug_mat, light_bars.at(lightbar_debug_idx).mid_p, \
                   3, Scalar(255, 255,255), -1);
            cout << "No." << lightbar_debug_idx << ": ";
            cout << "Angle: " << light_bars.at(lightbar_debug_idx).angle;
            cout << "Length: " << light_bars.at(lightbar_debug_idx).length << endl;
        }
        imshow("debug", lightbar_debug_mat);
        cout << "-----------------------------------------" << endl;
#endif
}


// Find Armors.
// Input image: Bin(coutours); Output image: RGB.
void ArmorMono::find_armor(Params params)
{
#define ARMOR_MONO_ARMOR_DEBUG

    ArmorROI_t armor_roi_temp = {};
#ifdef ARMOR_MONO_ARMOR_DEBUG
    cvNamedWindow("debug", CV_WINDOW_AUTOSIZE);
    Mat armor_debug_mat = Mat(720, 1280, CV_8UC3, Scalar(0, 0, 0));
#endif
    for(size_t lb_idx = 0; lb_idx < light_bars.size(); lb_idx++)
    {
        for(size_t lb_idx_idx = lb_idx + 1; lb_idx_idx < light_bars.size(); lb_idx_idx++)
        {
            // First based on the angle of lightbars' different to match them.
//            if(abs(light_bars.at(lb_idx).angle - light_bars.at(lb_idx_idx).angle) < params.armor_angle_diff_min)
            {
                // Then based on the length different to match them. (length > 0)
//                if((light_bars.at(lb_idx).length / light_bars.at(lb_idx_idx).length)    \
                                        < (1 + params.armor_length_retio_min)        || \
                    (light_bars.at(lb_idx).length / light_bars.at(lb_idx_idx).length)    \
                                        > (1 - params.armor_length_retio_min))
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
//                    rec_apex_remap(&(armor_roi_temp.apex_point[0]), armor_roi_temp.center);
//                    rec_apex_remap(&(armor_roi_temp.light_point[0]), armor_roi_temp.center);

                    armors.push_back(armor_roi_temp);
                }
            }
        }
    }

#ifdef ARMOR_MONO_ARMOR_DEBUG
//    cout << "-----------------------------------------" << endl;
//    cout << "                armor                    " << endl;
    for(size_t armor_idx = 0; armor_idx < armors.size(); armor_idx++)
    {
        for(size_t apex_idx = 0; apex_idx < 4; apex_idx++)
        {
            line(armor_debug_mat, armors.at(armor_idx).apex_point[apex_idx], \
                 armors.at(armor_idx).apex_point[(apex_idx + 1) % 4], \
                 Scalar(0, 0, 255), 1);
            line(armor_debug_mat, armors.at(armor_idx).light_point[apex_idx], \
                 armors.at(armor_idx).light_point[(apex_idx + 1) % 4], \
                 Scalar(0, 255, 255), 3);
            circle(armor_debug_mat, armors.at(armor_idx).center, \
                   5, Scalar(255, 255,255), -1);
        }
    }
//    cout << armors.size() << endl;
//    cout << "-----------------------------------------" << endl;
    imshow("debug", armor_debug_mat);
#endif
}

void ArmorMono::angle_slove(Params params)
{
//#define ARMOR_MONO_RANGING_DEBUG
#ifdef ARMOR_MONO_RANGING_DEBUG
    cvNamedWindow("debug", CV_WINDOW_AUTOSIZE);
    Mat ranging_debug_mat = Mat(720, 1280, CV_8UC3, Scalar(0, 0, 0));
    string target_distance = "distance: ";
    ostringstream target_distance_oss;
#endif
    Mat rvec, tvec;
    double x_err, y_err, distance;
    vector<Point2f> real_point, after_point;

    for(size_t real_p_idx = 0; real_p_idx < armors.size(); real_p_idx++)
    {
        real_point.clear();
        real_point.push_back(armors.at(real_p_idx).light_point[0]);
        real_point.push_back(armors.at(real_p_idx).light_point[1]);
        real_point.push_back(armors.at(real_p_idx).light_point[2]);
        real_point.push_back(armors.at(real_p_idx).light_point[3]);
        solvePnP(small_armor_world_point, real_point, params.mono_cam_matrix, \
                 params.mono_cam_distcoeffs, rvec, tvec, false, CV_ITERATIVE);
        x_err = atan(tvec.at<double>(0, 0) / tvec.at<double>(2, 0)) / 2 / CV_PI * 360;
        y_err = atan(tvec.at<double>(1, 0) / tvec.at<double>(2, 0)) / 2 / CV_PI * 360;
        distance = sqrt(tvec.at<double>(0, 0) * tvec.at<double>(0, 0) + \
                        tvec.at<double>(1, 0) * tvec.at<double>(1, 0) + \
                        tvec.at<double>(2, 0) * tvec.at<double>(2, 0));
#ifdef ARMOR_MONO_RANGING_DEBUG
        for(size_t proj_p_idx = 0; proj_p_idx < real_point.size(); proj_p_idx++)
        {
            circle(ranging_debug_mat, real_point[proj_p_idx], 3, Scalar(255, 255, 255), -1);
        }
        target_distance_oss << setiosflags(ios::fixed) << setprecision(3) << distance;
        target_distance = target_distance + target_distance_oss.str() + " cm.";
        putText(ranging_debug_mat, target_distance, Point(0, 10), \
                FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 255, 0), 1);
//        cout << distance << endl;
#endif
    }
#ifdef ARMOR_MONO_RANGING_DEBUG
    imshow("debug", ranging_debug_mat);
#endif
}

// Detect target.
void ArmorMono::target_detect(Params params)
{
    for(size_t armor_idx = 0; armor_idx < armors.size(); armor_idx++)
    {
        target_info.X_offset = armors.at(0).center.x - MONO_IMAGE_CENTER_X;
        target_info.Y_offset = armors.at(0).center.y - MONO_IMAGE_CENTER_Y;
//        target_info.X_offset = armors.at(0).center.x - MONO_IMAGE_CENTER_X;
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
    vector_clear();
//    range_cut(in_img, in_img);
    hsv_proc(in_img, in_img, params);
    fill_hole(in_img, in_img);
    GaussianBlur(in_img, in_img, Size(params.gauss_blur_coresize, params.gauss_blur_coresize), 0);
    oc_element = getStructuringElement(MORPH_RECT, Size(params.oc_oprt_coresize, \
                                                        params.oc_oprt_coresize));
    erode(in_img, in_img, oc_element);
    dilate(in_img, in_img, oc_element);
    find_lightbar(in_img, params);
    find_armor(params);
    angle_slove(params);
    target_detect(params);
}
