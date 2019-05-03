/*
 * @ File name: rune_monocular.cpp
 * @ Effect: Process images to hit the rune.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "inc/rune_monocular.h"

using namespace cv;
using namespace std;


// Used to pick up red and blue from low expusore image.
// Input image: RGB; Output image: Bin.
void RuneMono::hsv_proc(Mat &in_img, Mat &out_img, Params params)
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


void RuneMono::rune_mono_proc(Mat &in_img, Params params)
{
    vector<vector<Point>> fan_contours, target_contours;
    vector<Vec4i> fan_hierachy, target_hierachy;
    float contours_area = 0;
    Mat img = in_img.clone();

    hsv_proc(img, img, params);
    fill_hole(img, img);
    GaussianBlur(img, img, Size(params.rune_mono_proc_val.gauss_blur_coresize, \
                                params.rune_mono_proc_val.gauss_blur_coresize), 0);
    fan_oc_element = getStructuringElement(MORPH_RECT, \
                                           Size(params.rune_mono_proc_val.fan_oc_element_coresize, \
                                                params.rune_mono_proc_val.fan_oc_element_coresize));
    target_oc_element = getStructuringElement(MORPH_RECT, \
                                              Size(params.rune_mono_proc_val.target_oc_element_coresize, \
                                                   params.rune_mono_proc_val.target_oc_element_coresize));
    erode(img, img, fan_oc_element);
    dilate(img, img, fan_oc_element);
    findContours(img, fan_contours, fan_hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for (size_t contours_idx = 0; contours_idx < fan_contours.size(); contours_idx++)
    {
        fan_roi = minAreaRect(fan_contours[contours_idx]);
        fan_roi.points(fan_rect_p);
        fan_area = (float)(fan_roi.size.height * fan_roi.size.width);
        contours_area = (float)(contourArea(fan_contours[contours_idx]));
        if (fan_area > 20000.0f)
        {
            if ((contours_area / fan_area) < 0.6)
            {
//                rectangle(org_img, rectx, Scalar(0, 255, 0), 1);
                erode(img, img, target_oc_element);
                dilate(img, img, target_oc_element);
                findContours(img, target_contours, target_hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                for (size_t contours_idx_2 = 0; contours_idx_2 < target_contours.size(); contours_idx_2++)
                {
                    target_roi = minAreaRect(target_contours[contours_idx_2]);
                    target_area = (float)(target_roi.size.height * target_roi.size.width);
                    target_roi.points(target_rect_p);
//                        cout << "target size: " << target_area << endl;

                    if ((target_area > 5000.0f) && (target_area < 20000.0f))
                    {
                        //for (size_t i = 0; i < 4; i++)
                        //{
                        //	line(org_img, target_p[i], target_p[(i + 1) % 4], Scalar(0, 255, 255), 1);
                        //}
//                        circle(img, target.center, 8, Scalar(255, 0, 0), -1);
                    }
                }
            }
        }
    }
}
