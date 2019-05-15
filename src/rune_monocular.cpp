/*
 * @ File name: rune_monocular.cpp
 * @ Effect: Process images to hit the rune.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "inc/rune_monocular.h"
#include "sys/time.h"

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

    struct timeval tv1, tv2;
    struct timezone tz1, tz2;
    string debug_fps = "fps: ";
    string debug_proc_tim = "porc tim: ";
    string debug_gauss_blur_coresize = "gauss blur coresize: ";
    string debug_fan_oc_element_coresize = "oc fan coresize: ";
    string debug_tar_oc_element_coresize = "oc tar coresize: ";
    string debug_actived_fan_p_l = "actived fan's pixel number low: ";
    string debug_actived_fan_p_h = "actived fan's pixel number high: ";
    string debug_activing_fan_retio_l = "activing fan's pixel retio low: ";
    string debug_activing_fan_retio_h = "activing fan's pixel retio high: ";
    string debug_target_fan_retio_l = "target area's pixel retio low: ";
    string debug_target_fan_retio_h = "target area's pixel retio high: ";
    string debug_xy = "target X Y: ";
    string debug_target_flag = "target flag: ";
    float debug_proc_tim_val;

    Mat img = in_img.clone();
    Mat debug_img = in_img.clone();

    if(params.robo_cmd.debug_flag == true)
    {
        gettimeofday(&tv1, &tz1);
    }

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
    rune_target_info.target_flag = false;
    for (size_t contours_idx = 0; contours_idx < fan_contours.size(); contours_idx++)
    {
        fan_roi = minAreaRect(fan_contours[contours_idx]);
        fan_roi.points(fan_rect_p);
        fan_area = (float)(fan_roi.size.height * fan_roi.size.width);
        contours_area = (float)(contourArea(fan_contours[contours_idx]));
        if ((fan_area > params.rune_mono_proc_val.actived_fan_pixel_l) && \
            (fan_area < params.rune_mono_proc_val.actived_fan_pixel_h))
        {
            if (((contours_area / fan_area) < 0.6) && \
                ((contours_area / fan_area) > 0.2))
            {
                                            cout << contours_area / fan_area << endl;
                erode(img, img, target_oc_element);
                dilate(img, img, target_oc_element);
                findContours(img, target_contours, target_hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                for (size_t contours_idx_2 = 0; contours_idx_2 < target_contours.size(); contours_idx_2++)
                {
                    target_roi = minAreaRect(target_contours[contours_idx_2]);
                    target_area = (float)(target_roi.size.height * target_roi.size.width);
                    target_roi.points(target_rect_p);
                    cout << target_area << endl;
                    if ((target_area > 2500) && \
                        (target_area < 6000))
                    {
                        rune_target_info.X_offset = target_roi.center.x - frame_info.frame_mid_x;
                        rune_target_info.Y_offset = target_roi.center.y - frame_info.frame_mid_y;
                        rune_target_info.target_flag = true;
                    }
                    else
                    {
                        rune_target_info.X_offset = 0;
                        rune_target_info.Y_offset = 0;
                        rune_target_info.target_flag = false;
                    }
                }
            }
        }
    }
    if(rune_target_info.target_flag == false)
    {
        rune_target_info.X_offset = 0;
        rune_target_info.Y_offset = 0;
    }
    if(params.robo_cmd.debug_flag == true)
    {
        gettimeofday(&tv2, &tz2);
        debug_proc_tim_val = tv2.tv_sec * 1000 + tv2.tv_usec / 1000 - \
                             tv1.tv_sec * 1000 - tv1.tv_usec / 1000;
    }
    if(params.robo_cmd.debug_flag)
    {
        if(rune_target_info.target_flag == true)
        {
            for (size_t i = 0; i < 4; i++)
            {
                line(debug_img, target_rect_p[i], target_rect_p[(i + 1) % 4], Scalar(0, 255, 255), 1);
            }
            circle(debug_img, target_roi.center, 8, Scalar(255, 0, 0), -1);
        }
        debug_fps = debug_fps + to_string(params.mono_cam_val.mono_cam_far_FPS);
        putText(debug_img, debug_fps, Point(0, 15), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(255, 255, 0), 1.5);
        debug_proc_tim = debug_proc_tim + to_string(debug_proc_tim_val);
        putText(debug_img, debug_proc_tim, Point(0, 30), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(255, 255, 0), 1.5);
        debug_gauss_blur_coresize = debug_gauss_blur_coresize + to_string(params.rune_mono_proc_val.gauss_blur_coresize);
        putText(debug_img, debug_gauss_blur_coresize, Point(0, 45), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_fan_oc_element_coresize = debug_fan_oc_element_coresize + to_string(params.rune_mono_proc_val.fan_oc_element_coresize);
        putText(debug_img, debug_fan_oc_element_coresize, Point(0, 60), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_tar_oc_element_coresize = debug_tar_oc_element_coresize + to_string(params.rune_mono_proc_val.target_oc_element_coresize);
        putText(debug_img, debug_tar_oc_element_coresize, Point(0, 75), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_actived_fan_p_l = debug_actived_fan_p_l + to_string(params.rune_mono_proc_val.actived_fan_pixel_l);
        putText(debug_img, debug_actived_fan_p_l, Point(0, 90), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_actived_fan_p_h = debug_actived_fan_p_h + to_string(params.rune_mono_proc_val.actived_fan_pixel_h);
        putText(debug_img, debug_actived_fan_p_h, Point(0, 105), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_activing_fan_retio_l = debug_activing_fan_retio_l + to_string(params.rune_mono_proc_val.activing_fan_retio_l);
        putText(debug_img, debug_activing_fan_retio_l, Point(0, 120), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_activing_fan_retio_h = debug_activing_fan_retio_h + to_string(params.rune_mono_proc_val.activing_fan_retio_h);
        putText(debug_img, debug_activing_fan_retio_h, Point(0, 135), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_target_fan_retio_l = debug_target_fan_retio_l + to_string(params.rune_mono_proc_val.target_fan_retio_l);
        putText(debug_img, debug_target_fan_retio_l, Point(0, 150), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        debug_target_fan_retio_h = debug_target_fan_retio_h + to_string(params.rune_mono_proc_val.target_fan_retio_h);
        putText(debug_img, debug_target_fan_retio_h, Point(0, 165), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 0), 1.5);
        if(params.robo_cmd.enemy_color == RIN_ENEMY_RED)
        {
            putText(debug_img, "enemy red", Point(0, 180), \
                    DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 0, 285), 1.5);
        }
        else
        {
            putText(debug_img, "enemy blue", Point(0, 195), \
                    DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);
        }
        debug_xy = debug_xy + to_string((int16_t)rune_target_info.X_offset) + " " \
                            + to_string((int16_t)rune_target_info.Y_offset);
        putText(debug_img, debug_xy, Point(0, 210), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);
        debug_target_flag = debug_target_flag + to_string(rune_target_info.target_flag);
        putText(debug_img, debug_target_flag, Point(0, 225), \
                DEBUG_IAMGE_TEXT_FONT, 0.4, Scalar(0, 255, 255), 1.5);
        cvNamedWindow("rune_debug_image", CV_WINDOW_AUTOSIZE);
        imshow("rune_debug_image", debug_img);
        waitKey(1);
    }
    else
    {
        destroyAllWindows();
    }
}
