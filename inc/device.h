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
public:
    void serrial_cmd(void);
    void msg_send(void);
    void msg_read(void);
    void serial_run(void);
    void vision_init(void);
    bool is_open(void) const;
    void close();
private:
    void handle_rad(char buff[], boost::system::error_code ec, std::size_t bytes_transferred);
private:
    boost::asio::serial_port *_serial_port;
    char _pc_msg[15];
public:
    boost::asio::io_service serial_iosev;
    struct PCData {
        // General data:
        int16_t X_offset;   // -32767 - 32767.
        int16_t Y_offset;   // -32767 - 32767.
        int16_t X_KF;       // -32767 - 32767.
        int16_t Y_KF;       // -32767 - 32767.
        int16_t X_speed;    // -32767 - 32767.
        int16_t Y_speed;    // -32767 - 32767.
        int16_t Z_offset;   // 0 - 32767.
        uint8_t msg_get_flag;
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
        bool armor_type;    // Small armor:1, big armor: 0
    };
    RoboData robo_data;
    bool robo_cmdmsg_flag;
};


#endif // DEVICE_H
