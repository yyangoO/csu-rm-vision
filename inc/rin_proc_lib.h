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


struct Frame_t
{
    cv::Mat img;
    size_t idx;
    double time_stamp;
};

class FrameBuffer
{
public:
    FrameBuffer(size_t size);
    ~FrameBuffer(void) = default;
    bool push(const Frame_t& frame);
    bool get_latest(Frame_t& frame);
private:
    std::vector<Frame_t> _frames;
    std::vector<std::timed_mutex> _mutexs;
    size_t _tail_idx;
    size_t _head_idx;
    double _last_get_time_stamp;
};

class ImgProcCon
{
private:
    std::unique_ptr<RinVideoCapture> _mono_cap;
    std::unique_ptr<RinSerial> _rin_serial;
    std::unique_ptr<Params> _params;
    std::unique_ptr<ArmorMono> _armor_mono;
    std::unique_ptr<RuneMono> _rune_mono;
public:
    ImgProcCon(void);
    ~ImgProcCon(void) = default;
    void init(void);
    void img_get(void);
    void img_porc(void);
    void robo_cmd(void);
private:
    static void signal_handler(int);
    void init_signals(void);
private:
    static bool _quit_flag;
    FrameBuffer _frame_buffer;
    volatile uint8_t _task;
};


#endif // RIN_PROC_LIB_H
