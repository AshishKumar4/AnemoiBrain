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


#define CPACKET_MAGIC 110
#define REQ_SIGNAL     251
#define ACCEPT_SIGNAL    252
#define RPACKET_MAGIC  120
#define FALSE_PACKET   145

using namespace std;

ControlPackets defCp;
ResponsePackets rff;
ControlPackets *pp = &defCp;

int SPI_handshake()
{
    unsigned char ht = REQ_SIGNAL;
    SPI_ReadWrite(fd, (uintptr_t)&ht, (uintptr_t)&ht, 1);
    while(ht != ACCEPT_SIGNAL)
    {
        cout<<"Waiting for handshake with flight controller...\n";
    }
    return 1;
}

int IssueCommand()
{
    SPI_handshake();
    SPI_ReadWrite(fd, (uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void setThrottle(int throttle)
{
    unsigned char t = (unsigned char)throttle;
    pp->throttle = t;
    pp->magic = CP_MAGIC;
    IssueCommand();
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    pp->pitch = t;
    pp->magic = CP_MAGIC;
    IssueCommand();
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    pp->roll = t;
    pp->magic = CP_MAGIC;
    IssueCommand();
}

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    pp->yaw = t;
    pp->magic = CP_MAGIC;
    IssueCommand();
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

    /*pthread_t thread;
    
    //thread SPI_Updater_thread(SPI_Updater);
    if(pthread_create(&thread, NULL, SPI_Updater, 0))
    {
        cout<<"\nError creating threads...";
    }*/
    
    return 0;
}
