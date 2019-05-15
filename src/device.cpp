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
#include "inc/armor_monocular.h"
#include "inc/device.h"


#define SERIAL_BAUDRATE     (B115200)
#define DEVICE_PORT         "/dev/ttyUSB0"


#define RIN_PC_CMD     (0x03)
char rin_pc_fh[2] = {RIN_PC_CMD, ~RIN_PC_CMD};
char rin_pc_ft[2] = {~RIN_PC_CMD, RIN_PC_CMD};
#define RIN_ROBO_CMD   (0x30)
char rin_robo_fh[2] = {RIN_ROBO_CMD, ~RIN_ROBO_CMD};
char rin_robo_ft[2] = {~RIN_ROBO_CMD, RIN_ROBO_CMD};


using namespace std;
using namespace cv;


// Open serial.
void RinSerial::serrial_cmd(void)
{
    _serial_fd = open(DEVICE_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_serial_fd == -1)
         cout << "serial open failed" << endl;
    tcgetattr(_serial_fd, &_opt);
    cfmakeraw(&_opt);
    cfsetispeed(&_opt, SERIAL_BAUDRATE);
    cfsetospeed(&_opt, SERIAL_BAUDRATE);
    tcsetattr(_serial_fd, TCSANOW, &_opt);
    tcflush(_serial_fd, TCIOFLUSH);
    if(_serial_fd == -1)
        cout << "serial set failed" << endl;
}

// Message send.
void RinSerial::msg_send(void)
{
    int write_l[3];
    _pc_msg[0] = _pc_data.X_offset & 0xff;
    _pc_msg[1] = _pc_data.X_offset >> 8;
    _pc_msg[2] = _pc_data.Y_offset & 0xff;
    _pc_msg[3] = _pc_data.Y_offset >> 8;
    _pc_msg[4] = _pc_data.Z_offset & 0xff;
    _pc_msg[5] = _pc_data.Z_offset >> 8;
    _pc_msg[6] = _pc_data.target_flag;
    tcflush(_serial_fd, TCOFLUSH);
    write_l[0] = write(_serial_fd, (void *)rin_pc_fh, sizeof(rin_pc_fh) / sizeof(char));
    write_l[1] = write(_serial_fd, (void *)_pc_msg, sizeof(_pc_msg) / sizeof(char));
    write_l[2] = write(_serial_fd, (void *)rin_pc_ft, sizeof(rin_pc_ft) / sizeof(char));
    if((write_l[0] != sizeof(rin_pc_fh) / sizeof(char)) ||  \
       (write_l[0] != sizeof(rin_pc_fh) / sizeof(char)) ||  \
       (write_l[0] != sizeof(rin_pc_fh) / sizeof(char)))
        cout << "serial data wrong" << endl;
    if(_serial_fd == -1)
        cout << "serial write failed" << endl;
}

// Vision read.
void RinSerial::vision_init(void)
{
    char buff[5];
    int send_l;
    while(1)
    {
        memset(buff, 0, 5);
        send_l = read(_serial_fd, buff, 5);
        if(send_l == 5)
        {
            if((buff[0] == rin_robo_fh[0]) && \
               (buff[1] == rin_robo_fh[1]) && \
               (buff[3] == rin_robo_ft[0]) && \
               (buff[4] == rin_robo_ft[1]))
            {
                RinSerial::robo_data.init_flag = (buff[2] >> 7) & 0x01;
                RinSerial::robo_data.set_flag = (buff[2] >> 6) & 0x01;
                RinSerial::robo_data.debug_flag = (buff[2] >> 5) & 0x01;
                RinSerial::robo_data.enemy_color = (buff[2] >> 4) & 0x01;
                RinSerial::robo_data.aim_or_rune = (buff[2] >> 3) & 0x01;
                RinSerial::robo_data.armor_type = (buff[2] >> 2) & 0x01;

                if((RinSerial::robo_data.init_flag == true) &&
                    (RinSerial::robo_data.set_flag == false))
                    break;
            }
        }
    }
}

// Message read.
void RinSerial::msg_read(void)
{
    char buff[5];
    int send_l;
    memset(buff, 0, 5);
    send_l = read(_serial_fd, buff, 5);
    if(send_l == 5)
    {
        if((buff[0] == rin_robo_fh[0]) && \
           (buff[1] == rin_robo_fh[1]) && \
           (buff[3] == rin_robo_ft[0]) && \
           (buff[4] == rin_robo_ft[1]))
        {
            RinSerial::robo_data.init_flag = (buff[2] >> 7) & 0x01;
            RinSerial::robo_data.set_flag = (buff[2] >> 6) & 0x01;
            RinSerial::robo_data.debug_flag = (buff[2] >> 5) & 0x01;
            RinSerial::robo_data.enemy_color = (buff[2] >> 4) & 0x01;
            RinSerial::robo_data.aim_or_rune = (buff[2] >> 3) & 0x01;
            RinSerial::robo_data.armor_type = (buff[2] >> 2) & 0x01;
        }
    }
}
