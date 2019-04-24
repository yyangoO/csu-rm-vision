/*
 * @ File name: device.cpp
 * @ Effect: Serial using.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#include <iostream>
#include <stdio.h>
#include <chrono>
#include "inc/device.h"
#include "inc/armor_monocular.h"


#define SERIAL_BAUDRATE     (115200)
#define DEVICE_PORT         "/dev/ttyUSB0"


#define RIN_PC_CMD     (0x03)
char rin_pc_fh[2] = {RIN_PC_CMD, ~RIN_PC_CMD};
char rin_pc_ft[2] = {~RIN_PC_CMD, RIN_PC_CMD};
#define RIN_ROBO_CMD   (0x30)
char rin_robo_fh[2] = {RIN_ROBO_CMD, ~RIN_ROBO_CMD};
char rin_robo_ft[2] = {~RIN_ROBO_CMD, RIN_ROBO_CMD};


using namespace std;
using namespace cv;
using namespace boost::asio;


// Open serial.
void RinSerial::serrial_cmd(void)
{
    _serial_port = new serial_port(serial_iosev, DEVICE_PORT);
    _serial_port->set_option(serial_port::baud_rate(SERIAL_BAUDRATE));
    _serial_port->set_option(serial_port::flow_control(serial_port::flow_control::none));
    _serial_port->set_option(serial_port::parity(serial_port::parity::none));
    _serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    _serial_port->set_option(serial_port::character_size(8));
}

// Message send.
void RinSerial::msg_send(void)
{
    _pc_msg[0] = _pc_data.X_offset & 0xff;
    _pc_msg[1] = _pc_data.X_offset >> 8;
    _pc_msg[2] = _pc_data.Y_offset & 0xff;
    _pc_msg[3] = _pc_data.Y_offset >> 8;
    _pc_msg[4] = _pc_data.X_KF & 0xff;
    _pc_msg[5] = _pc_data.X_KF >> 8;
    _pc_msg[6] = _pc_data.Y_KF & 0xff;
    _pc_msg[7] = _pc_data.Y_KF >> 8;
    _pc_msg[8] = _pc_data.X_speed & 0xff;
    _pc_msg[9] = _pc_data.X_speed >> 8;
    _pc_msg[10] = _pc_data.Y_speed & 0xff;
    _pc_msg[11] = _pc_data.Y_speed >> 8;
    _pc_msg[12] = _pc_data.Z_offset & 0xff;
    _pc_msg[13] = _pc_data.Z_offset >> 8;
    if(robo_cmdmsg_flag == true)
        _pc_msg[14] = 0xff;
    else
        _pc_msg[14] = 0x00;
    write(*_serial_port, buffer((void *)rin_pc_fh, sizeof(rin_pc_fh) / sizeof(char)));       // Frame header.
    write(*_serial_port, buffer((void *)_pc_msg, sizeof(_pc_msg) / sizeof(char)));           // Data.
    write(*_serial_port, buffer((void *)rin_pc_ft, sizeof(rin_pc_ft) / sizeof(char)));       // Frame tail.
}

// Vision read.
void RinSerial::vision_init(void)
{
    char buff[5];
//    while(1)
//    {
//        memset(buff, 0, 5);
//        read(*_serial_port, buffer(buff));
//        if((buff[0] == rin_robo_fh[0]) && \
//           (buff[1] == rin_robo_fh[1]) && \
//           (buff[3] == rin_robo_ft[0]) && \
//           (buff[4] == rin_robo_ft[1]))
//        {
            RinSerial::robo_data.init_flag = (buff[2] >> 7) & 0x01;
            RinSerial::robo_data.set_flag = (buff[2] >> 6) & 0x01;
            RinSerial::robo_data.debug_flag = (buff[2] >> 5) & 0x01;
            RinSerial::robo_data.enemy_color = (buff[2] >> 4) & 0x01;
            RinSerial::robo_data.aim_or_rune = (buff[2] >> 3) & 0x01;
            RinSerial::robo_data.reso_flag = (buff[2] >> 2) & 0x01;
            RinSerial::robo_data.armor_type = (buff[2] >> 1) & 0x01;

            RinSerial::robo_data.init_flag = 0;
            RinSerial::robo_data.set_flag = 0;
            RinSerial::robo_data.debug_flag = 1;
            RinSerial::robo_data.enemy_color = 1;
            RinSerial::robo_data.aim_or_rune = 0;
            RinSerial::robo_data.reso_flag = 0;
            RinSerial::robo_data.armor_type = 0;
//            break;
//        }
//        serial_iosev.run();
//    }
}

// Message read.
void RinSerial::msg_read(void)
{
    uint8_t buff[5];
//    const auto t1 = chrono::high_resolution_clock::now();
    memset(buff, 0, 5);
//    read(*_serial_port, buffer(buff));
    serial_iosev.run();
}
