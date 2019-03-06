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


//using namespace std;
using namespace cv;


RinSerial rin_serial;

// Open serial.
void RinSerial::serrial_cmd(void)
{
    uid_t uid;
    int fd;
    const char *path = "/dev/ttyS0";
    struct termios port_settins;

    // Get root.
    uid = getuid();
    if(setuid(0)){}

    // Open port.
    fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    {
        perror("Error: Serrial port open failed!\n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
        printf("@: Serrial port open succeed!\n");
    }
    // Configure port.
    cfsetispeed(&port_settins, B115200);
    cfsetospeed(&port_settins, B115200);
    port_settins.c_cflag &= ~PARENB;
    port_settins.c_cflag &= ~CSTOPB;
    port_settins.c_cflag &= ~CSIZE;
    port_settins.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &port_settins);
    if(fd == -1)
    {
        perror("Error: Serrial port configuretion failed!\n");
    }
    else
    {
        printf("@: Serrial port configure succeed!\n");
    }
}

// Serrial write.
void RinSerial::data_send(const void *data)
{
    int fd;
    int x;
    x = 1024;
    try
    {
        write(fd, (void *)&x, sizeof(x));
//        std::cout << sizeof(x) << std::endl;
    }
    catch(Exception e)
    {
        std::cout << e.what() << std::endl;
    }
//    if(fd == -1)
//    {
//        perror("XError: Serrial port tranform failed!\n");
//    }
}
// PC message set
