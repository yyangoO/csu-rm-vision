/*
 * @ File name: device.cpp
 * @ Effect: includes.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef INCLUDES_H
#define INCLUDES_H


#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#define DEBUG_IAMGE_TEXT_FONT   FONT_HERSHEY_SIMPLEX
#define DEBUG_IMAGE_TEXT_PERAL  (15)

#define RIN_ENEMY_RED   true
#define RIN_ENEMY_BLUE  false
#define RIN_AIM         false
#define RIN_RUNE        true
#define RIN_BIG_ARMOR   false
#define RIN_SMALL_ARMOR true


const cv::Point world_origin_point = cv::Point2f(0.0f, 0.0f);


void fill_hole(cv::Mat &in_img, cv::Mat &out_img);
void calibration_img_get(void);


#endif // INCLUDES_H
