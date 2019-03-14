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


#define INFANTRY_MODE
//#define HERO_MODE
//#define SENTRY_MODE


#define DEBUG
//#define RELEASE

#define DEBUG_IAMGE_TEXT_FONT   FONT_HERSHEY_SIMPLEX
#define DEBUG_IMAGE_TEXT_PERAL  (15)

#define ENEMY_RED   1
#define ENEMY_BLUE  0

//#define MONO_IMAGE_X_SIZE   1280
//#define MONO_IMAGE_Y_SIZE   720
//#define MONO_IMAGE_CENTER_X (639.0f)
//#define MONO_IMAGE_CENTER_Y (359.0f)
#define MONO_IMAGE_X_SIZE   640
#define MONO_IMAGE_Y_SIZE   480
#define MONO_IMAGE_CENTER_X (319.0f)
#define MONO_IMAGE_CENTER_Y (239.0f)
#define FPS 120

const cv::Point world_origin_point = cv::Point2f(0.0f, 0.0f);


void fill_hole(cv::Mat &in_img, cv::Mat &out_img);
void calibration_img_get(void);


#endif // INCLUDES_H
