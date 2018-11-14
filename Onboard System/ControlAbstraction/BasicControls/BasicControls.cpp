#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
//#include <wiringPiSPI.h>
#include <string>
#include <thread> // std::thread

#include <pthread.h>

#include "BasicControls.h"

/*
    There are two possible configurations, 
    1) RPI unit is on board the drone and communicates with 
        flight controller through SPI, and so the telemetry unit is connected to RPI directly.
    2) RPI unit is off board the drone and communicates with the flight controller through Radio (wifi/ Xbee)
*/
//#define ONBOARD_SPI
#define OFFBOARD_RADIO

/*
    Telemetry Type
*/
#define NRF24
//#define WIFI 
//#define XBEE

#define CPACKET_MAGIC 110
#define REQ_SIGNAL     251
#define ACCEPT_SIGNAL    252
#define RPACKET_MAGIC  120
#define FALSE_PACKET   145

using namespace std;

ControlPackets defCp;
ResponsePackets rff;
ControlPackets *pp = &defCp;

#ifdef defined ONBOARD_SPI 

#include "SPIdrivers.c"

int fd;

int IssueCommand()
{
    SPI_handshake();
    SPI_ReadWrite(fd, (uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void Raw_Init()
{
    fd = SPI_init("/dev/spidev0.0");
}

#elif defined OFFBOARD_RADIO

#include "RadioDrivers.cpp"

    #if defined NRF24
int IssueCommand()
{
    NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void Raw_Init()
{
    
}
    #endif
#endif 



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

int BasicControls_init()
{
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    Raw_Init();

    pp->magic = 110;
    pp->throttle = 0;
    pp->pitch = 0;
    pp->roll = 0;
    pp->yaw = 0;

    //SPI_handshake();
    IssueCommand();

    /*pthread_t thread;
    
    //thread SPI_Updater_thread(SPI_Updater);
    if(pthread_create(&thread, NULL, SPI_Updater, 0))
    {
        cout<<"\nError creating threads...";
    }*/
    
    return 0;
}
