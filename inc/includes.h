#ifndef INCLUDES_H
#define INCLUDES_H


#include "sys/time.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "inc/parameters.h"


#define INFANTRY_1_MODE
//#define INFANTRY_2_MODE
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

#define DEBUG_IAMGE_TEXT_FONT   FONT_HERSHEY_SIMPLEX


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


//extern RMVideoCapture cap_mono;
const cv::Point world_origin_point = cv::Point2f(0.0f, 0.0f);


void fill_hole(cv::Mat &in_img, cv::Mat &out_img);
void calibration_img_get(void);


#endif // INCLUDES_H
