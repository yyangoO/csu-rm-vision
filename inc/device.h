#ifndef DEVICE_H
#define DEVICE_H

#include "termios.h"
#include "inc/includes.h"
#include "inc/parameters.h"


class RinSerial
{
public:
    void serrial_cmd(void);
    void msg_set(void *data, int data_type);
    void msg_get(void);
    void close();
//private:
    void data_send(const void *data);
    void data_read(void);
private:
    struct PCData {
        // General data:
        bool bullet_type;               // Small bullet: 0, big bullet 1.
        bool suggest_shutfreq_flag;     // Do not have enough infromation: 0.
        bool suggest_shutspeed_freq;    // Do not have enough information: 0.
        bool shut_flag;                 // Shut: 1, Not shut: 0.
        int X_offset;                   // -32767 - 32767.
        int Y_offset;                   // -32767 - 32767.
        int shut_freq;                  // 0 - 256.
        int shut_speed;                 // 0 - 256.
        int err_type;
        // Infantry data:
        bool aim_or_rune;               // Aim: 0, Rune: 1.
        // Hero data:

        // Sentry data:
        int Enemy_type;                 // Infantry: 0, Hero: 1, Engineer: 2.
        int Enemy_distance;             // 0 - 256; --> 0m - 5m.
    };
    PCData pc_data;
    struct RobotData {

    };
    RobotData robot_data;
    int _serial_fd;
    struct termios opt;
};


extern RinSerial rin_serial;


#endif // DEVICE_H
