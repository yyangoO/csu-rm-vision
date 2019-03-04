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

#ifdef DEBUG
#undef RELEASE
#endif
#ifdef RELEASE
#undef DEBUG
#endif


#define ENEMY_RED   1
#define ENEMY_BLUE  0


const cv::Point world_origin_point = cv::Point2f(0.0f, 0.0f);


void fill_hole(cv::Mat &in_img, cv::Mat &out_img);
void rec_apex_remap(cv::Point2f *rec_apex, cv::Point2f rec_mid_point);


#endif // INCLUDES_H
