#pragma once
#include "iostream"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

class SerialPort
{
    int fd;
    std::string port;

  public:
    SerialPort(std::string device, int baudrate, int method = FNDELAY);
    int open_port(char *port, int baudrate = 115200, int methods = FNDELAY);
    int Write(char *buff, int size);
    int Read(char *buff, int size);
};