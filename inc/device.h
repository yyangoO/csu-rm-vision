/*
 * @ File name: device.h
 * @ Effect: Serial using.
 * @ Author: CSU Yangyang
 * @ Last change date: 2019/1/10
 */


#ifndef DEVICE_H
#define DEVICE_H

#include <termios.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/serial_port.hpp>
#include "inc/includes.h"
#include "inc/parameters.h"


class RinSerial
{
//public:
//    RinSerial(const any_type &port_name);
//    ~RinSerial(void);
//    void msg_sned(void);
//    void msg_read(void);
//    void handle_read(char buff
public:
    void serrial_cmd(void);
    void msg_send(void);
    void msg_read(void);
    bool is_open(void) const;
    void close();
private:
    void handle_read(uint8_t buff[], boost::system::error_code ec, std::size_t bytes_transferred);
private:
//    int _serial_fd;
//    struct termios _opt;
    boost::asio::serial_port *_serial_port;
    uint8_t _pc_msg[14];
public:
    boost::asio::io_service serial_iosev;
    struct PCData {
        // General data:
        int16_t X_offset;   // -32767 - 32767.
        int16_t Y_offset;   // -32767 - 32767.
        int16_t X_KF;       // -32767 - 32767.
        int16_t Y_KF;       // -32767 - 32767.
        int16_t X_speed;   // -32767 - 32767.
        int16_t Y_speed;   // -32767 - 32767.
        int16_t Z_offset;   // 0 - 32767.
        // Sentry data:
        bool shut_flag;     // Shut: 1, Not shut: 0.
        int enemy_type;     // Infantry: 0, Hero: 1, Engineer: 2.
    };
    PCData _pc_data;
    struct RoboData {
        bool init_flag;     // Init:1, don't init 0.
        bool set_flag;      // Set: 1, don't set 0.
        bool debug_flag;    // Debug: 1, don't debug 0.
        bool enemy_color;   // Red: 1, blue: 0.
        bool aim_or_rune;   // Aim: 0, Rune: 1.
        bool reso_flag;     // Close: 1, far: 0.
        bool bullet_type;   // Small bullet: 0, big bullet 1.
        uint8_t robo_type;  // Infantry: 0, Hero: 1, Sentry:2.
    };
    RoboData robo_data;
};


#endif // DEVICE_H
