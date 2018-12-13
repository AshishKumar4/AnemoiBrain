#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
//#include <wiringPiSPI.h>
#include <string>
#include <thread> // std::thread
#include <unistd.h>
#include <pthread.h>

#include "BasicControls.h"

/*
    There are two possible configurations, 
    1) RPI unit is on board the drone and communicates with 
        flight controller through SPI, and so the telemetry unit is connected to RPI directly.
    2) RPI unit is off board the drone and communicates with the flight controller through Radio (wifi/ Xbee)
*/
#define ONBOARD_SPI
//#define OFFBOARD_RADIO

/*
    Telemetry Type
*/
//#define NRF24
//#define WIFI
//#define XBEE

using namespace std;

ControlPackets defCp;
ControlPackets oldDefCp;
ResponsePackets rff;
ControlPackets *pp = &defCp;
ControlPackets *ppold = &oldDefCp;

timespec *t100n = new timespec;
timespec *t1000n = new timespec;
timespec *t10000n = new timespec;
timespec *t100000n = new timespec;

#ifdef ONBOARD_SPI

#include "SPI/SPIdrivers.h"

int fd;

#define TIMEOUT_VAL 20

/*
    The new handshake mechanism works like this ->
    Send a special msg, then wait for an amount of time for it to arrive. 
    if it arrives, validate, else print handshake failed or timed out
*/
int SPI_handshake()
{
back:
    int ht = REQ_SIGNAL;
    int hb = ht;
    SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1);
    // We first need to tell the FC that we are ready to send handshake!
    for (int i = 0; i < TIMEOUT_VAL; i++)
    {
        if (ht != hb) // Value was actually Successfully updated
        {
            /*if (ht == ACCEPT_SIGNAL)
            {
                goto proceed;
            }
            cout << "Handshake Failed, wrong value recieved [" << ht << "]";
            return 1;*/
        }
        nanosleep(t100n, NULL);
    }
    cout << "Handshake Timed out...";
    return 1;

proceed:
    ht = REQ2_SIGNAL;
    SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1);
    return 0;
}

unsigned char checksum(unsigned char *buf, int len)
{
    unsigned char tt = 0;
    for (int i = 0; i < len - 1; i++)
    {
        tt ^= buf[i];
    }
    return tt;
}

int IssueCommand()
{
    if (!SPI_handshake())
    {
        uint8_t *ht = ((uint8_t *)pp);
        uint8_t *hr = ((uint8_t *)&rff);
        pp->checksum = checksum(ht, sizeof(ControlPackets));
        uint8_t tb = 0;
        for (int i = 0; i < sizeof(ControlPackets); i++)
        {
        back:
            *hr = 0;
            tb = *hr;
            SPI_ReadWrite((int)fd, (uintptr_t)ht, (uintptr_t)hr, (size_t)1);
            for (int j = 0; j < TIMEOUT_VAL; j++)
            {
                if (*hr != tb) // Value was actually Successfully updated
                {
                    goto suc;
                }
                nanosleep(t100n, NULL);
            }
            cout << "[NOP " << i << "]";
            goto back;
        suc:
            nanosleep(t100n, NULL);
            ++ht;
            ++hr;
        }
        cout << "Successfully Issued Command\n";
        return 0;
    }
    return 1;
}

void *SPI_Updater(void *threadid)
{
    cout << "\nSPI Updater Initialized...";
    uint8_t *tbo = (uint8_t *)ppold;
    uint8_t *tb = (uint8_t *)pp;
    while (1)
    {
        /*
            Well, Basically, Firstly, check if the new data that we are gonna send is identical to the older, we don't need to send anything!
        */
        for (int i = 0; i < sizeof(ControlPackets); i++)
        {
            if (tb[i] != tbo[i])
                goto update;
        }
        goto skip;
    update:
        memcpy((void *)tbo, (void *)tb, sizeof(ControlPackets));
        if (IssueCommand())
        {
            cout << "Couldn't Issue the command\n";
        }
        //SPI_ReadWrite(fd, (uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
        //wiringPiSPIDataRW(0, (unsigned char*)pp, sizeof(ControlPackets));
    skip:
        nanosleep(t10000n, NULL);
    }
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
    //IssueCommand();
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    pp->pitch = t;
    pp->magic = CP_MAGIC;
    //IssueCommand();
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    pp->roll = t;
    pp->magic = CP_MAGIC;
    //IssueCommand();
}

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    pp->yaw = t;
    pp->magic = CP_MAGIC;
    //IssueCommand();
}

void setAux1(int val)
{
    unsigned char t = (unsigned char)val;
    pp->aux1 = t;
    pp->magic = CP_MAGIC;
    //IssueCommand();
}

void setAux2(int val)
{
    unsigned char t = (unsigned char)val;
    pp->aux2 = t;
    pp->magic = CP_MAGIC;
    //IssueCommand();
}

ResponsePackets *getResponse()
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

    t100n->tv_sec = 0;
    t100n->tv_nsec = 100;
    t1000n->tv_sec = 0;
    t1000n->tv_nsec = 1000;
    t10000n->tv_sec = 0;
    t10000n->tv_nsec = 10000;
    t100000n->tv_sec = 0;
    t100000n->tv_nsec = 100000;

    //SPI_handshake();
    //IssueCommand();

    pthread_t thread;

    //thread SPI_Updater_thread(SPI_Updater);
    if (pthread_create(&thread, NULL, SPI_Updater, 0))
    {
        cout << "\nError creating threads...";
    }

    return 0;
}
