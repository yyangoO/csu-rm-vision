#include "inc/device.h"


#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>


#define SERIAL_BAUDRATE B115200
#define DEVICE_PORT "/dev/ttyUSB0"


using namespace std;
using namespace cv;


RinSerial rin_serial;

// Open serial.
void RinSerial::serrial_cmd(void)
{
//    setuid(getuid());
//    setuid(getgid());
    _serial_fd = open(DEVICE_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_serial_fd == -1)
    {
        perror("@!Err: (port open failed)");
    }
    tcgetattr(_serial_fd, &opt);
    cfmakeraw(&opt);
    cfsetispeed(&opt, SERIAL_BAUDRATE);
    cfsetospeed(&opt, SERIAL_BAUDRATE);
    tcsetattr(_serial_fd, TCSANOW, &opt);
    if(_serial_fd == -1)
    {
        perror("@!Err(baudrate set failed)");
    }
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~INPCK;
    opt.c_cflag &= (SERIAL_BAUDRATE | CLOCAL | CREAD);
    opt.c_cflag &= ~(INLCR | ICRNL);
    opt.c_cflag &= ~(IXON);
    opt.c_lflag &- ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~ OPOST;
    opt.c_oflag &= ~(ONLCR | OCRNL);
    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);
    opt.c_cc[VTIME] = 1;
    opt.c_cc[VMIN] = 1;
    tcflush(_serial_fd, TCIOFLUSH);
    if(_serial_fd == -1)
    {
        perror("@!Err(databits set failed)");
    }
    printf("@: Serial CMD succeed!\n");
}

// Serial write.
void RinSerial::data_send(const void *data)
{
    tcflush(_serial_fd, TCOFLUSH);
    if(write(_serial_fd, data, sizeof(data)) != sizeof(data))
    {
        perror(("@!Err(data write failed)"));
    };
    if(_serial_fd == -1)
    {
        perror(("@!Err(data send failed)"));
    }
}

//Serial read.
void RinSerial::data_read(void)
{
    char *x;
    read(_serial_fd, (void *)x, sizeof(x));
    tcflush(_serial_fd, TCIFLUSH);
    cout << (char *)x << endl;
}
