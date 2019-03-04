#ifndef DEVICE_H
#define DEVICE_H


#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include "inc/includes.h"
#include "inc/parameters.h"


//class RinSerial
//{
//public:
//    void serial_set(const std::string, const OpenOptions = defaultOptions);
//    void open(const std::string path = "/dev/ttyS0");
//    bool is_open(void) const;
//    int write(const void *data, int length);
//    int read(void *data, int length);
//    void close();
//private:
//    void baudrate_set(int fd, unsigned long baudrate);
//    void serial_set(int fd);
//protected:
//    termios_options(termios &tios, const OpenOptions &options);
//public:
//    static std::vector<std::string> list();
//private:
//    struct termios opt;
//    static const OpenOptions defaultOptions;
//    std::string path;
//    int tty_fd;
//    bool is_open_flag;
//};


//bool operator == (const SerialOpt::OpenOptions & lhs, const SerialOpt::OpenOptions &rhs);
//bool operator != (const SerialOpt::OpenOptions & lhs, const SerialOpt::OpenOptions &rhs);


//extern RinSerial rin_serial;


#endif // DEVICE_H
