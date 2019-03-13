/*
 * @ File name: rin_proc_lib.h
 * @ Effect: My persional ways to process images.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef RIN_PROC_LIB_H
#define RIN_PROC_LIB_H


#include <mutex>
#include <string>

#include "inc/includes.h"
#include "inc/rin_videocapture.h"
#include "inc/parameters.h"
#include "inc/rin_videocapture.h"
#include "inc/armor_monocular.h"
#include "inc/rune_monocular.h"
#include "inc/parameters.h"
#include "inc/device.h"


class ImgProcCon
{
private:
    std::unique_ptr<RinVideoCapture> _mono_cap;
    std::unique_ptr<RinSerial> _rin_serial;
    std::unique_ptr<Params> _params;
    std::unique_ptr<ArmorMono> _armor_mono;
    std::unique_ptr<RuneMono> _rune_mono;
public:
    ImgProcCon();
private:
    const std::string _cam_params_path;
};


extern ImgProcCon img_porc_con;

#endif // RIN_PROC_LIB_H
