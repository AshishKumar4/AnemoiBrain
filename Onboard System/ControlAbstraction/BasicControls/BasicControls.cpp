#include <sys/ioctl.h>
#include "SPIdrivers.c"
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

#include <string>

using namespace std;

struct ControlPackets
{
    char throttle;
    char pitch;
    char roll;
    char yaw;
    char aux1;
    char aux2;
    char switches;
    char random[9];
};

int main()
{
    ControlPackets* pp = new ControlPackets;
    int fd = SPI_init("/dev/spidev0.0");
    while(1)
    {
        SPI_ReadWrite(fd, (uintptr_t)pp, sizeof(ControlPackets));
        cout<<pp->random;
        //strcpy(msg, "Hello Arduino!!!");
        usleep(20);
    }
}