/*
 * @ File name: device.cpp
 * @ Effect: Serial using.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


//#include <unistd.h>
//#include <fcntl.h>
//#include <termios.h>
#include <iostream>
#include <stdio.h>
//#include <stdlib.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <errno.h>
#include "inc/device.h"
#include "inc/armor_monocular.h"


#define SERIAL_BAUDRATE 115200
#define DEVICE_PORT "/dev/ttyUSB0"


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
//    _serial_fd = open(DEVICE_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
//    if(_serial_fd == -1)
//    {
//        return;
//    }
//    tcgetattr(_serial_fd, &_opt);
//    cfmakeraw(&_opt);
//    cfsetispeed(&_opt, SERIAL_BAUDRATE);
//    cfsetospeed(&_opt, SERIAL_BAUDRATE);
//    tcsetattr(_serial_fd, TCSANOW, &_opt);
//    if(_serial_fd == -1)
//    {
//        return;
//    }
//    tcflush(_serial_fd, TCIOFLUSH);
//    if(_serial_fd == -1)
//    {
//        return;
//    }
//    else
//    {
//        return;
//    }
}

void RinSerial::handle_read(uint8_t buff[], boost::system::error_code ec, std::size_t bytes_transferred)
{
    cout << bytes_transferred << endl;
    cout << "serial read" << endl;
    serial_iosev.reset();
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
    write(*_serial_port, buffer((void *)rin_pc_fh, sizeof(rin_pc_fh) / sizeof(char)));       // Frame header.
    write(*_serial_port, buffer((void *)_pc_msg, sizeof(_pc_msg) / sizeof(char)));           // Data.
    write(*_serial_port, buffer((void *)rin_pc_ft, sizeof(rin_pc_ft) / sizeof(char)));       // Frame tail.
}

// Message read.
void RinSerial::msg_read(void)
{
    uint8_t buff[5];
//    deadline_timer timer(serial_iosev);
//    timer.expires_from_now(boost::posix_time::millisec(1));
    memset(buff, 0, 5);
    async_read(*_serial_port, buffer(buff), boost::bind(&RinSerial::handle_read, this, buff, _1, _2));
//    timer.async_wait(boost::bind(&serial_port::cancel, boost::ref(_serial_port)));
//    read(*_serial_port, buffer(buff));
//    printf("%x %x %x %x %x\n", buff[0], buff[1], buff[2], buff[3], buff[4]);
//    printf("%d\n", buff[0]);
//    cout << buff[0] << endl;
    serial_iosev.run();
}
