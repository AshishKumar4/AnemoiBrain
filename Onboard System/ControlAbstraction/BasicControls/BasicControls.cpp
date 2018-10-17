#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "SPIdrivers.c"
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <wiringPiSPI.h>
#include <string>

using namespace std;

#define CP_MAGIC 110

int checksum(char *buf, int len)
{
    char tt = 0;
    for (int i = 0; i < len - 1; i++)
    {
        tt ^= buf[i];
    }
    return tt;
}

struct ControlPackets
{
    unsigned char magic;
    unsigned char throttle;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char aux1;
    unsigned char aux2;
    unsigned char switches;
    unsigned char random[9];
    unsigned char checksum;
};

ControlPackets defCp;
ControlPackets ref;
ControlPackets *pp=&defCp;

void setThrottle(int throttle)
{
    unsigned char t = (unsigned char)throttle;
    pp->throttle = t;
    pp->magic = CP_MAGIC;
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    pp->pitch = t;
    pp->magic = CP_MAGIC;
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    pp->roll = t;
    pp->magic = CP_MAGIC;
}

void setYar(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    pp->yaw = t;
    pp->magic = CP_MAGIC;
}

int main()
{
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    int fd = SPI_init("/dev/spidev0.0");

    pp->magic = 110;
    pp->throttle = 255;
    pp->pitch = 230;
    pp->roll = 20;
    pp->yaw = 40;
    while (1)
    {
        setThrottle(240);
        //pp->checksum = checksum((char*)pp, sizeof(ControlPackets));
        SPI_ReadWrite(fd, (uintptr_t)pp, (uintptr_t)&ref, sizeof(ControlPackets));
        //wiringPiSPIDataRW(0, (unsigned char*)pp, sizeof(ControlPackets));
        usleep(200);
    }
}
