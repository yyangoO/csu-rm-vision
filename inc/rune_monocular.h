/*
 * @ File name: rune_monocular.h
 * @ Effect: Process images to hit the rune.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef RUNE_MONOCULAR_H
#define RUNE_MONOCULAR_H


#include <vector>

#include "inc/includes.h"
#include "inc/parameters.h"


class RuneMono {
public:
    void rune_mono_proc(cv::Mat &in_img, Params params);
private:
    void hsv_proc(cv::Mat &in_img, cv::Mat &out_img, Params params);
};

#endif // RUNE_MONOCULAR_H
