/*
 * @ File name: rin_proc_lib.h
 * @ Effect: My persional ways to process images and add threads.(Based on Vision by SEU).
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef RIN_PROC_LIB_H
#define RIN_PROC_LIB_H


#include <mutex>
#include <string>
#include <vector>
#include "inc/includes.h"
#include "inc/parameters.h"
#include "inc/rin_videocapture.h"
#include "inc/armor_monocular.h"
#include "inc/rune_monocular.h"
#include "inc/parameters.h"
#include "inc/device.h"


class ImgPorcCon
{
private:
    RinVideoCapture _mono_cap;
    RinSerial _rin_serial;
    Params _params;
    ArmorMono _armor_mono;
    RuneMono _rune_mono;
public:
    void init(void);
    void info_get(void);
    void img_proc(void);
    void robo_cmd(void);
    void vision_run(void);
private:
    cv::Mat _mono_img;
};


#endif // RIN_PROC_LIB_H
