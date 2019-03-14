/*
 * @ File name: device.cpp
 * @ Effect: Serial using.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include "inc/device.h"
#include "inc/armor_monocular.h"


#define SERIAL_BAUDRATE B115200
#define DEVICE_PORT "/dev/ttyUSB0"


#define VISION_INFO_CMD     (0x01)
char vision_info_fh[2] = {VISION_INFO_CMD, ~VISION_INFO_CMD};
char vision_info_ft[2] = {~VISION_INFO_CMD, VISION_INFO_CMD};

#ifdef INFANTRY_MODE
#define INFANTRY_PC_CMD     (0x03)
char infantry_pc_fh[2] = {INFANTRY_PC_CMD, ~INFANTRY_PC_CMD};
char infantry_pc_ft[2] = {~INFANTRY_PC_CMD, INFANTRY_PC_CMD};
#define INFANTRY_ROBO_CMD   (0x21)
char infantry_robo_fh[2] = {INFANTRY_ROBO_CMD, ~INFANTRY_ROBO_CMD};
char infantry_robo_ft[2] = {~INFANTRY_ROBO_CMD, INFANTRY_ROBO_CMD};
#endif

#ifdef HERO_MODE
#define HERO_PC_CMD         (0x12)
#define HERO_ROBO_CMD       (0x22)
#endif

#ifdef SENTRY_MODE
#define SENTRY_PC_CMD       (0x13)
#define SENTRY_ROBO_CMD     (0x23)
#endif

using namespace std;
using namespace cv;


RinSerial rin_serial;


// Open serial.
void RinSerial::serrial_cmd(void)
{
    _serial_fd = open(DEVICE_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_serial_fd == -1)
    {
        return;
    }
    tcgetattr(_serial_fd, &_opt);
    cfmakeraw(&_opt);
    cfsetispeed(&_opt, SERIAL_BAUDRATE);
    cfsetospeed(&_opt, SERIAL_BAUDRATE);
    tcsetattr(_serial_fd, TCSANOW, &_opt);
    if(_serial_fd == -1)
    {
        return;
    }
    tcflush(_serial_fd, TCIOFLUSH);
    if(_serial_fd == -1)
    {
        return;
    }
    else
    {
        return;
    }
}

// Message send.
void RinSerial::msg_send(void)
{
#ifdef INFANTRY_MODE
    _pc_data.X_offset = (short int)(armor_mono.target_info.X_offset/* / MONO_IMAGE_X_SIZE * 2 * 32767*/);
    _pc_data.Y_offset = (short int)(armor_mono.target_info.Y_offset/* / MONO_IMAGE_Y_SIZE * 2 * 32767*/);
    cout << _pc_data.X_offset << "  " << _pc_data.Y_offset/* << endl*/;
    _infantry_pc_msg[0] = _pc_data.X_offset >> 8;
    _infantry_pc_msg[1] = _pc_data.X_offset & 0xff;
    _infantry_pc_msg[2] = _pc_data.Y_offset >> 8;
    _infantry_pc_msg[3] = _pc_data.Y_offset & 0xff;
//    _infantry_pc_msg[4] = _pc_data.Z_offset;
    write(_serial_fd, (void *)infantry_pc_fh, sizeof(infantry_pc_fh) / sizeof(char));       // Frame header.
    write(_serial_fd, (void *)_infantry_pc_msg, sizeof(_infantry_pc_msg) / sizeof(char));   // Data.
    write(_serial_fd, (void *)infantry_pc_ft, sizeof(infantry_pc_ft) / sizeof(char));       // Frame tail.
#endif

#ifdef HERO_MODE
    _pc_data.X_offset = (short int)(armor_mono.target_info.X_offset/* / MONO_IMAGE_X_SIZE * 2 * 32767*/);
    _pc_data.Y_offset = (short int)(armor_mono.target_info.Y_offset/* / MONO_IMAGE_Y_SIZE * 2 * 32767*/);
    cout << _pc_data.X_offset << "  " << _pc_data.Y_offset/* << endl*/;
    _infantry_pc_msg[0] = _pc_data.X_offset >> 8;
    _infantry_pc_msg[1] = _pc_data.X_offset & 0xff;
    _infantry_pc_msg[2] = _pc_data.Y_offset >> 8;
    _infantry_pc_msg[3] = _pc_data.Y_offset & 0xff;
    _infantry_pc_msg[4] = _pc_data.Z_offset;
    write(_serial_fd, (void *)infantry_pc_fh, sizeof(infantry_pc_fh) / sizeof(char));       // Frame header.
    write(_serial_fd, (void *)_infantry_pc_msg, sizeof(_infantry_pc_msg) / sizeof(char));   // Data.
    write(_serial_fd, (void *)infantry_pc_ft, sizeof(infantry_pc_ft) / sizeof(char));       // Frame tail.
#endif

#ifdef SENTRY_MODE

#endif
}

// Message read.
void RinSerial::msg_read(void)
{
#ifdef INFANTRY_1_MODE

#endif

#ifdef INFANTRY_2_MODE

#endif

#ifdef HERO_MODE

#endif

#ifdef SENTRY_MODE

#endif
}
