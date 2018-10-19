#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "SPIdrivers.c"
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
//#include <wiringPiSPI.h>
#include <string>
#include <thread> // std::thread

#include <pthread.h>

#include "BasicControls.h"

using namespace std;

ControlPackets defCp;
ResponsePackets rff;
ControlPackets *pp = &defCp;

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

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    pp->yaw = t;
    pp->magic = CP_MAGIC;
}

ResponsePackets* getResponse()
{
    return &rff;
}

int fd;

void *SPI_Updater(void *threadid)
{
    cout<<"\nSPI Updater Initialized...";
    while (1)
    {
        SPI_ReadWrite(fd, (uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
        //strcpy((char*)rff, "[Done]");
        //wiringPiSPIDataRW(0, (unsigned char*)pp, sizeof(ControlPackets));
        usleep(5);
    }
}

int BasicControls_init()
{
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    fd = SPI_init("/dev/spidev0.0");

    pp->magic = 110;
    pp->throttle = 0;
    pp->pitch = 0;
    pp->roll = 0;
    pp->yaw = 0;

    pthread_t thread;
    
    //thread SPI_Updater_thread(SPI_Updater);
    if(pthread_create(&thread, NULL, SPI_Updater, 0))
    {
        cout<<"\nError creating threads...";
    }
    
    return 0;
}
