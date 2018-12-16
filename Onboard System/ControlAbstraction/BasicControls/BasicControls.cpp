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
{
    int ht = REQ2_SIGNAL;
#ifndef GROUND_TEST_NO_FC
    SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1); //wiringPiSPIDataRW(0, (unsigned char*)&ht, 1);
#endif
    //nanosleep(t100000n, NULL);
    return 0;
}

int IssueCommand()
{
    if (!SPI_handshake())
    {
        uint8_t *ht = ((uint8_t *)pp);
        uint8_t *hr = ((uint8_t *)&rff);
        pp->checksum = checksum(ht, sizeof(ControlPackets));
        uint8_t tb = 0;
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

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    // First send the value, then the channel, then a dummy and check the returned checksum.
    // if its good, okay otherwise repeat.
    mtx.lock();
    uint8_t tv, tc, bv, bc;
    uint8_t flg1 = 0, flg2 = 0;
    int counter = 0;
back:
    printf("\n[Attempting send %d to %d", val, channel);
    bv = val;
    bc = channel;

    wiringPiSPIDataRW(0, &bv, 1); // Send the value
    nanosleep(t10000n, NULL);

    wiringPiSPIDataRW(0, &bc, 1); // Send the channel, recieve the value
    nanosleep(t10000n, NULL);

    uint8_t tmp1, tmp2;

    if (bc == val ^ 0xff || flg1)
    {
        printf(" [correct val] ");
        tmp1 = 0x0f;
        flg1 = 1;
    }
    else
    {
        printf(" [false val %d] ", bc);
        tmp1 = 0xf0;
        flg1 = 0;
    }

    wiringPiSPIDataRW(0, &tmp1, 1); // Send confirmation of reception, recieve channel
    nanosleep(t10000n, NULL);

    if (tmp1 == channel ^ 0xff || flg2)
    {
        printf(" [correct channel] ");
        tmp2 = 0x0f;
        flg2 = 1;
    }
    else
    {
        printf(" [false channel %d] ", bc);
        tmp2 = 0xf0;
        flg2 = 0;
    }

    wiringPiSPIDataRW(0, &tmp2, 1); // Send confirmation of reception, recieve something
    nanosleep(t10000n, NULL);

    /*
        If the value was wrong but channel was right, send the value only;
        and vice versa
    */
    if (flg1 & flg2)
    {
        printf("\tSuccess!");
        mtx.unlock();
        return;
    }
    else
    {
        printf("\t[Retrying]");
        goto back;
    }
}

/*
static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    // First send the value, then the channel, then a dummy and check the returned checksum.
    // if its good, okay otherwise repeat.
    mtx.lock();
    uint8_t tv, tc, bv, bc;
    uint8_t tl = tc ^ tv;
    int counter = 0;
back:
    printf("\n[Attempting send %d to %d", val, channel);
    tv = val;
    tc = channel;
    bv = val;
    bc = channel;

#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &bv, 1);
#endif
    nanosleep(t10000n, NULL);
chnl:
#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &bc, 1);
#endif
    nanosleep(t10000n, NULL);
    // The ACK for value must have arrived in bc
chk:
    bv = tl;
#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &bv, 1);
    // The ACK for channel (checksum) in bv
#endif
    nanosleep(t10000n, NULL);

    uint8_t tk = bv;

    if(val != bc)
    {
        bv = 0;     // Send a wrong Checksum
    }

#ifndef GROUND_TEST_NO_FC
    wiringPiSPIDataRW(0, &bv, 1);
    // send the checksum back
#endif
    nanosleep(t10000n, NULL);

#ifndef GROUND_TEST_NO_FC
    if (bc == val)
    {
        if (tk == tl)
        {
        done:
            printf("\t[SUCCESS %d, %d]", bv, tl);
            mtx.unlock();
            return;
        }
        else if (counter < 20)
        {
            printf("\tFailed! expected [%d], got [%d], Retrying", tl, tk);
            ++counter;
            goto back;
        }
        else
        {
            printf("\nTimed Out!");
        }
    }
    else if (counter < 20)
    {
        printf("\tFailed! 2 Expected [%d], got [%d], Retrying", val, bc);
        ++counter;
        goto back;
    }
    mtx.unlock();
    return;
}
    /*
recheck:
    if (bc == val)
    {
        if (tk == tl) // Checksum
        {
        done:
            printf("\t[SUCCESS %d, %d]", tk, tl);
            mtx.unlock();
            return;
        }
        else if (!tk)
        {
            for (int i = 0; i < 10; i++)
            {
                if (tk == tl)
                    goto done;
                nanosleep(t10000n, NULL);
            }
            ++counter;
            goto back;
        }
    }
    else
    {
        printf("[bc != val, %d %d]", bc, val);
    }
    if (counter < 10)
    {
        printf("\tFailed! expected [%d], got [%d], Retrying", tl, tk);
        ++counter;
        goto back;
    }
    else
    {
        printf("\n<<TIMED-OUT!!!>>");
        counter = 0;
        bv = REQ2_SIGNAL;
        /*
#ifndef GROUND_TEST_NO_FC
        wiringPiSPIDataRW(0, &bv, 1);
#endif
        nanosleep(t100n, NULL);
        mtx.unlock();
        return;
    }
    printf("\nShouldn't have come here");
#endif
    mtx.unlock();
}*/

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
