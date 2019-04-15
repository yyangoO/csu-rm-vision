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
    if(params.BR_HSV_range.enemy_color == RIN_ENEMY_RED)
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
    else if(params.BR_HSV_range.enemy_color == RIN_ENEMY_BLUE)
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
    Mat img;
    VideoCapture cap("../csu_rm_vision_v2.1/samples/big_rune.mp4");
    cap >> img;
    cvNamedWindow("org", CV_WINDOW_AUTOSIZE);
    imshow("org", img);
    if(img.empty())
    {
        return;
    }
    hsv_proc(img, img, params);
    cvNamedWindow("rune", CV_WINDOW_AUTOSIZE);
    imshow("rune", img);
    waitKey(40);
}
