#include "iostream"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "Serial.hpp"

SerialPort::SerialPort(std::string device, int baudrate, int method)
{
    fd = open_port((char *)device.c_str(), baudrate, method);
}

int SerialPort::open_port(char *port, int baudrate, int methods)
{
    int ff; /* File descriptor for the port */

    ff = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (ff == -1)
    {
        /*
    * Could not open the port.
    */

        perror("open_port: Unable to open ");
        perror(port);
    }
    else
        fcntl(ff, F_SETFL, methods);
    /*
    * Get the current options for the port...
    */

    struct termios options;
    tcgetattr(ff, &options);

    /*
    * Set the baud rates to 115200...
    */

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /*
    * Enable the receiver and set local mode...
    */

    //options.c_cflag |= (CLOCAL | CREAD);

    /*
    * Set the new options for the port...
    */

    tcsetattr(ff, TCSANOW, &options);
    /*char* buf[100];
    read(fd, buf, 100);
    printf("[%s]", buf);*/
    return (ff);
}

int SerialPort::Write(char *buff, int size)
{
    //printf("{<%s>}", buff);
    int n = write(fd, buff, size);
    if (n < 0)
    {
        //fputs("write() of 'x' bytes failed!\n", stderr);
        printf("\nwrite() of %d bytes failed!\n, {%s}", size, buff);
    }
    return n;
}

int SerialPort::Read(char *buff, int size)
{
    int bb = read(fd, buff, size);
    return bb;
}