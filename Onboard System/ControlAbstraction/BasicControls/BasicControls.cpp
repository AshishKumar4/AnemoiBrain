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
#include <mutex>

#include "BasicControls.h"

//#define GROUND_TEST_NO_FC

/*
    There are two possible configurations, 
    1) RPI unit is on board the drone and communicates with 
        flight controller through SPI, and so the telemetry unit is connected to RPI directly.
    2) RPI unit is off board the drone and communicates with the flight controller through Radio (wifi/ Xbee)
*/
#define ONBOARD_SPI
//#define OFFBOARD_RADIO

#define SYNCD_TRANSFER

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

#ifndef GROUND_TEST_NO_FC
#include "SPI/SPIdrivers.h"
#include "wiringPiSPI.h"
#endif

int fd;

#define TIMEOUT_VAL 20

/*
    The new handshake mechanism works like this ->
    Send a special msg, then wait for an amount of time for it to arrive. 
    if it arrives, validate, else print handshake failed or timed out
*/

unsigned char checksum(unsigned char *buf, int len)
{
    unsigned char tt = 0;
    for (int i = 0; i < len - 1; i++)
    {
        tt += buf[i];
    }
    return tt;
}

int SPI_handshake()
{ /*
back:
    int ht = REQ_SIGNAL;
    int hb = ht;
#ifndef GROUND_TEST_NO_FC
    //SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1);
    wiringPiSPIDataRW(0, (unsigned char*)&ht, 1);
#endif
    nanosleep(t100000n, NULL);
    // We first need to tell the FC that we are ready to send handshake!
    for (int i = 0; i < TIMEOUT_VAL; i++)
    {
        if (ht != hb) // Value was actually Successfully updated
        {
            cout<<"[Got "<<ht<<"]";
            goto proceed;
            //if (ht == ACCEPT_SIGNAL)
            //{
            //    goto proceed;
            //}
            cout << "Handshake Failed, wrong value recieved [" << ht << "]";
            return 1;
        }
        nanosleep(t10000n, NULL);
    }
    cout << "Handshake Timed out...";
    return 1;

proceed:*/
    int ht = REQ2_SIGNAL;
#ifndef GROUND_TEST_NO_FC
    SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1); //wiringPiSPIDataRW(0, (unsigned char*)&ht, 1);
#endif
    //nanosleep(t100000n, NULL);
    return 0;
    /*for (int i = 0; i < TIMEOUT_VAL; i++)
    {
        if (ht != hb) // Value was actually Successfully updated
        {
            cout<<"[Final Got "<<ht<<"]";
            return 0;
            if (ht == ACCEPT_SIGNAL)
            {
                goto proceed;
            }
            cout << "Handshake Failed, wrong value recieved [" << ht << "]";
            return 1;
        }
        nanosleep(t10000n, NULL);
    }
    cout << "Handshake Timed out at stage 2...";
    return 1;*/
}

int IssueCommand()
{
    if (!SPI_handshake())
    {
        uint8_t *ht = ((uint8_t *)pp);
        uint8_t *hr = ((uint8_t *)&rff);
        pp->checksum = checksum(ht, sizeof(ControlPackets));
        uint8_t tb = 0;
        /*
        for (int i = 0; i < sizeof(ControlPackets); i++)
        {
        back:
            //tb = *hr;
#ifndef GROUND_TEST_NO_FC
            SPI_ReadWrite((int)fd, (uintptr_t)ht, (uintptr_t)hr, (size_t)1);
            //wiringPiSPIDataRW(0, (unsigned char*)ht, 1);
#endif
            nanosleep(t100000n, NULL);
            for (int j = 0; j < TIMEOUT_VAL; j++)
            {
                if (*hr != tb) // Value was actually Successfully updated
                {
                    goto suc;
                }
                nanosleep(t100000n, NULL);
            }
            cout << "[NOP " << i << "]";
            goto back;
        suc:
            nanosleep(t100000n, NULL);
            ++ht;
            ++hr;
        }*/

#ifndef GROUND_TEST_NO_FC
        SPI_ReadWrite((int)fd, (uintptr_t)ht, (uintptr_t)hr, (size_t)sizeof(ControlPackets));
#endif
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
    skip:
        nanosleep(t10000n, NULL);
    }
}

void Raw_Init()
{
#ifndef GROUND_TEST_NO_FC
    //fd = SPI_init("/dev/spidev0.0");
    fd = wiringPiSPISetup(0, 1000000);
#endif
}

mutex mtx;
volatile CommandPackets cp;
/*
static volatile void sendCommand(uint8_t val, int channel)
{
#ifdef SYNCD_TRANSFER
    mtx.lock();
back:
    cp.magic = (uint8_t)REQ2_SIGNAL;
    cp.value = (uint8_t)val;
    cp.channel = (uint8_t)channel;
    cp.checksum = cp.value + cp.channel;
    printf("\n[Sending Command %d to %d, %d]", val, channel, sizeof(CommandPackets));

#ifndef GROUND_TEST_NO_FC
    //SPI_ReadWrite((int)fd, (uintptr_t)&cp, (uintptr_t)&cp, (size_t)sizeof(CommandPackets));
    //wiringPiSPIDataRW(0, (unsigned char*)&cp, sizeof(CommandPackets));
#endif
    uint8_t *ht = (uint8_t *)&cp;
    for (int i = 0; i < 4; i++)
    {
#ifndef GROUND_TEST_NO_FC
        wiringPiSPIDataRW(0, ht, 1);
#endif
        nanosleep(t10000n, NULL);
        ++ht;
    }
// A Dummy transfer to verify if everything went all-right
retry_ack:
    ht = new uint8_t;

#ifndef GROUND_TEST_NO_FC
        wiringPiSPIDataRW(0, ht, 1);
#endif
    nanosleep(t100000n, NULL);

#ifdef GROUND_TEST_NO_FC
    *ht = ACK_GOT_PACKET; // Fool the system to give as real a simulation as possible
#endif

    if (*ht == ACK_GOT_PACKET)
    {
        printf("\t[ACK]Command Issued Successfully!");
    }
    else if (*ht == FALSE_PACKET)
    {
        printf("\t[Packet Lost]");
        goto back;
    }
    else
    {
        printf("\tSomething Unexpected!");
        goto retry_ack;
    }
    //nanosleep(t10000n, NULL);
    mtx.unlock();
#endif
}
*/

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    // First send the value, then the channel, then a dummy and check the returned checksum.
    // if its good, okay otherwise repeat.
back:
    printf("\n[Attempting send %d to %d", val, channel);
    uint8_t tv = val, tc = channel;
#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &val, 1);
#endif
    nanosleep(t1000n, NULL);

#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &channel, 1);
#endif
    nanosleep(t1000n, NULL);

#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &val, 1);
#endif
    nanosleep(t1000n, NULL);

    if (val != tc + tv)
    {
        printf("\tFailed!, Retrying");
        goto back;
    }
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For Nrf24L01 Communication------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

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
    sendCommand(throttle, 0);
    //IssueCommand();
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    pp->pitch = t;
    pp->magic = CP_MAGIC;
    sendCommand(pitch, 1);
    //IssueCommand();
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    pp->roll = t;
    pp->magic = CP_MAGIC;
    sendCommand(roll, 2);
    //IssueCommand();
}

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    pp->yaw = t;
    pp->magic = CP_MAGIC;
    sendCommand(yaw, 3);
    //IssueCommand();
}

void setAux1(int val)
{
    unsigned char t = (unsigned char)val;
    pp->aux1 = t;
    pp->magic = CP_MAGIC;
    sendCommand(val, 5);
    //IssueCommand();
}

void setAux2(int val)
{
    unsigned char t = (unsigned char)val;
    pp->aux2 = t;
    pp->magic = CP_MAGIC;
    sendCommand(val, 5);
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

    pp->magic = CP_MAGIC;
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
#ifndef SYNCD_TRANSFER
    //thread SPI_Updater_thread(SPI_Updater);
    if (pthread_create(&thread, NULL, SPI_Updater, 0))
    {
        cout << "\nError creating threads...";
    }
#endif
    return 0;
}
