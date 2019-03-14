/*
 * @ File name: rin_proc_lib.cpp
 * @ Effect: My persional ways to process images and add threads.(Based on Vision by SEU).
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include <math.h>
#include <mutex>
#include <signal.h>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include "inc/rin_proc_lib.h"


using namespace std;
using namespace cv;


FrameBuffer::FrameBuffer(size_t size) : _frames(size), \
                                        _mutexs(size), \
                                        _tail_idx(0), \
                                        _head_idx(0), \
                                        _last_get_time_stamp(0.0) {}

bool FrameBuffer::push(const Frame_t &frame)
{
    const size_t new_head_idx = (_head_idx + 1) % _frames.size();   // Get new head.
    // Try for 1ms to lock.
    unique_lock<timed_mutex> lock(_mutexs[new_head_idx], chrono::milliseconds(1));
    if(!lock.owns_lock())
    {
        return false;
    }
    _frames[new_head_idx] = frame;
    if(new_head_idx == _tail_idx)
    {
        _tail_idx = (_tail_idx + 1) % _frames.size();
    }
    _head_idx = new_head_idx;
    return true;
}

bool FrameBuffer::get_latest(Frame_t &frame)
{
    volatile const size_t head_idx = _head_idx;
    // Try for 1ms to lock.
    unique_lock<timed_mutex> lock(_mutexs[head_idx], chrono::milliseconds(1));
    if(!lock.owns_lock() || _frames[head_idx].img.empty() || \
                            _frames[head_idx].time_stamp == _last_get_time_stamp)
    {
        return false;
    }
    frame = _frames[head_idx];
    _last_get_time_stamp = _frames[head_idx].time_stamp;
    return true;
}

ImgProcCon::ImgProcCon(void) : _mono_cap(make_unique<RinVideoCapture>()), \
                               _frame_buffer(6), \
                               _rin_serial(make_unique<RinSerial>()), \
                               _params(make_unique<Params>()), \
                               _armor_mono(make_unique<ArmorMono>()), \
                               _rune_mono(make_unique<RuneMono>()),\
                               _task() {}



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

// Calibration
void calibration_img_get(void)
{
    int idx = 0;
    RinVideoCapture mono_cap("/dev/video0", 3);
    Mat img;
    string path;
    mono_cap.set_format(MONO_IMAGE_X_SIZE, MONO_IMAGE_Y_SIZE, 1);
    mono_cap.set_FPS(FPS);
    mono_cap.set_exposure_time(false, 8);
    mono_cap.start_stream();
    while(1)
    {
        mono_cap >> img;
        cvNamedWindow("calibration_image", CV_WINDOW_AUTOSIZE);
        imshow("calibration_image", img);
        if(waitKey(1) != -1)
        {
            idx++;
            path = "/home/csu-rm-infantry-1/csu_rm_vision/vision_by_rinck/csu_rm_vision_v2.1/data/";
            path = path + to_string(idx);
            path = path + ".jpg";
            imwrite(path, img);
        }
    }
}
