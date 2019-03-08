#ifndef DEVICE_H
#define DEVICE_H

#include "termios.h"
#include "inc/includes.h"
#include "inc/parameters.h"


class RinSerial
{
public:
    void serrial_cmd(void);
    void msg_send(void);
    void msg_read(void);
    bool is_open(void) const;
    void close();
private:
    int _serial_fd;
    struct termios _opt;
    struct PCData {
        // General data:
        short int X_offset;         // -32767 - 32767.
        short int Y_offset;         // -32767 - 32767.
        unsigned char Z_offset;     // 0 - 255: 0 - 8m
        int err_type;
        // Infantry data:
        bool aim_or_rune;       // Aim: 0, Rune: 1.
        // Hero data:
        bool bullet_type;       // Small bullet: 0, big bullet 1.
        // Sentry data:
        bool shut_flag;         // Shut: 1, Not shut: 0.
        int Enemy_type;         // Infantry: 0, Hero: 1, Engineer: 2.
        int Enemy_distance;     // 0 - 256; --> 0m - 5m.
    };
    PCData _pc_data;
    struct RoboData {

    };
    RoboData robo_data;
    char _infantry_pc_msg[5];
    char _hero_pc_msg[5];
};


extern RinSerial rin_serial;


#endif // DEVICE_H
