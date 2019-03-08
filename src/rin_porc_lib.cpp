/*
 * @ File name: rin_proc_lib.cpp
 * @ Effect: My persional ways to process images.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include "math.h"
#include "inc/includes.h"


using namespace cv;


// Arm to fill the lightbar's hole.
// Input image: Bin; Output image: Bin.
void fill_hole(Mat &in_img, Mat &out_img)
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

// Remap the points' sort as clockwise.
// Warning: Only deal with convex polygon.
void rec_apex_remap(Point2f *rec_apex, Point2f rec_mid_point)
{
//    float slope_temp, angle[4];
//    Point sort_temp;
//    for(size_t point_idx = 0; point_idx < 4; point_idx++)
//    {
//        slope_temp = ((rec_apex + point_idx)->y - rec_mid_point.y) /   \
//                     ((rec_apex + point_idx)->x - rec_mid_point.x);
//        angle[point_idx] = atan(slope_temp);
////        angle[point_idx] = angle[point_idx] + 2 * CV_PI;
//    }
//    for(size_t sort_idx = 0; sort_idx < 3; sort_idx++)
//    {
//        if(angle[sort_idx] < angle[sort_idx + 1])
//        {
//            sort_temp = rec_apex[sort_idx];
//            rec_apex[sort_idx] = rec_apex[sort_idx + 1];
//            rec_apex[sort_idx + 1] = sort_temp;
//        }
//        else if(angle[sort_idx] = angle[sort_idx + 1])
//        {

//        }
//    }
    Point2f point_differ[4];
    Point2f org_point[4];
    for(size_t point_idx = 0; point_idx < 4; point_idx++)
    {
        org_point[point_idx] = *(rec_apex + point_idx);
    }
    for(size_t point_idx = 0; point_idx < 4; point_idx++)
    {
        point_differ[point_idx].x = (org_point + point_idx)->x - rec_mid_point.x;
        point_differ[point_idx].y = (org_point + point_idx)->y - rec_mid_point.y;
        if((point_differ[point_idx].x > 0) && (point_differ[point_idx].y > 0))
        {
            *rec_apex = *(rec_apex + point_idx);
        }
        else if((point_differ[point_idx].x > 0) && (point_differ[point_idx].y < 0))
        {
            *(rec_apex + 1) = *(rec_apex + point_idx);
        }
        else if((point_differ[point_idx].x < 0) && (point_differ[point_idx].y < 0))
        {
            *(rec_apex + 2) = *(rec_apex + point_idx);
        }
        else if((point_differ[point_idx].x < 0) && (point_differ[point_idx].y > 0))
        {
            *(rec_apex + 3) = *(rec_apex + point_idx);
        }
    }
}
