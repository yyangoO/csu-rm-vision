#include "inc/device.h"


//using namespace cv;


//RinSerial rin_serial;

//// Open serial.
//void RinSerial::open(const std::string path = "/dev/ttyS0")
//{
//    int fd;
//    fd = std::open(path, O_RDWR | O_NOCTTY | O_NDELAY);
//    if(fd == -1)
//    {
//        is_open_flag = false;
//        perror("Error: Serial Open failed!\n");
//    }
//    else
//    {
//        fcntl(fd, F_SETFL, 0);
//        printf("Serial port open succeed!\n");
//    }
//}

//// Check is open or not.
//bool RinSerial::is_open() const
//{
//    return is_open_flag;
//}

//// Set baud rate.
//void RinSerial::baudrate_set(int fd, unsigned long baudrate)
//{
//    int status;
//    tcgetattr(fd, &opt);
//    tcflush(fd, TCIFLUSH);
//    cfsetispeed(&opt, baudrate);
//    cfsetospeed(&opt, baudrate);
//    status = tcsetattr(fd, TCSANOW, &opt);
//    if(status != 0)
//    {
//        perror("Error: Tcsetattr fd1\n");
//        return;
//    }
//    tcflush(fd, TCIFLUSH);
//}

//// Set data bits.
//void set_data_bits(int fd, int data_bits)
//{
//    if(tcgetattr(fd, &opt) != 0)
//    {
//        perror("Error: Set data bits failed!\n");
//        return;
//    }
//    cfsetispeed()

//}
