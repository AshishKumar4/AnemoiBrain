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
#include <mutex>

#include "BasicControls.h"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
    Two Modes -> 
        1. Aisim simulation mode, fake, simple flight controller using Airsim C++ APIs 
        2. Real Drone Mode, To fly the real thing. Real Flight Controller required.
*/
//#define MODE_AIRSIM
//#define MODE_MAVLINK_SIM
//#define MODE_DEBUG_NO_FC

#if !defined(MODE_AIRSIM) && !defined(MODE_MAVLINK_SIM) && !defined(MODE_DEBUG_NO_FC)
#define MODE_REALDRONE
#endif

#define SYNCD_TRANSFER
#define UPDATER_THREAD

/*
    Outputs to be shown on CLI
*/

#define SHOW_STATUS_RC
#define SHOW_STATUS_ARMED

#if defined(MODE_REALDRONE)
/*
        There are two possible configurations, 
        1) RPI unit is on board the drone and communicates with 
            flight controller, and so the telemetry unit is connected to RPI directly.
        2) RPI unit is off board the drone and communicates with the flight controller through Radio (wifi/ Xbee)
    */

/*
        Telemetry Protocol
    */
//#define ONBOARD_SPI_PROTOCOL
//#define NRF24L01_SPI_PROTOCOL
//#define I2C_PROTOCOL
#define MSP_Serial_PROTOCOL

/*
        Data Gathering method
    */
#define MSP_SERIAL_CLI_MONITOR

/*
        Telemetry Type
    */
//#define NRF24
//#define WIFI
//#define Xbee
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------------Some Definitions----------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX1 4
#define AUX2 5

uint8_t RC_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
using namespace std;
mutex mtx;

#if defined(ONBOARD_SPI_PROTOCOL) || defined(NRF24L01_SPI_PROTOCOL) || defined(I2C_PROTOCOL)
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
    //unsigned char random[9];
    unsigned char checksum;
};

struct ResponsePackets
{
    unsigned char magic;
    unsigned char alt;
    unsigned char pitch;
    unsigned char roll;
    unsigned char yaw;
    unsigned char lat;
    unsigned char lon;
    unsigned char heading;
    //unsigned char random[9];
    unsigned char checksum;
};

struct CommandPackets
{
    uint8_t magic;
    uint8_t value;
    uint8_t channel;
    uint8_t checksum;
};

ControlPackets defCp;
ControlPackets oldDefCp;
ResponsePackets rff;
ControlPackets *pp = &defCp;
ControlPackets *ppold = &oldDefCp;
#else

#endif

timespec *t100n = new timespec;
timespec *t1000n = new timespec;
timespec *t10000n = new timespec;
timespec *t100000n = new timespec;

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For SPI Based Communication------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(ONBOARD_SPI_PROTOCOL)

#ifndef MODE_DEBUG_NO_FC
#include "LowLevel/SPIdrivers.h"
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
#ifndef MODE_DEBUG_NO_FC
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
#ifndef MODE_DEBUG_NO_FC
        SPI_ReadWrite((int)fd, (uintptr_t)ht, (uintptr_t)hr, (size_t)sizeof(ControlPackets));
#endif
        cout << "Successfully Issued Command\n";
        return 0;
    }
    return 1;
}

void Channel_Updater(int threadid)
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

void Raw_Init(int argc, char *argv[])
{
#ifndef MODE_DEBUG_NO_FC
    //fd = SPI_init("/dev/spidev0.0");
    fd = wiringPiSPISetup(0, 500000);
#endif
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    // First send the value, then the channel, then a dummy and check the returned checksum.
    // if its good, okay otherwise repeat.
    mtx.lock();
    uint8_t *bv = new uint8_t;
    uint8_t *bc = new uint8_t;
    int flg1 = 0, flg2 = 0;
    uint8_t *tmp1 = new uint8_t;
    uint8_t *tmp2 = new uint8_t;
    int counter = 0;
    uint8_t *t1 = new uint8_t;
    uint8_t *t2 = new uint8_t;
back:
    printf("\n[Attempting send %d to %d <%d, %d>", val, channel, flg1, flg2);
    *bv = val;
    *bc = channel;

#ifndef MODE_DEBUG_NO_FC
    wiringPiSPIDataRW(0, bv, 1); // Send the value
#endif
    nanosleep(t100000n, NULL);

#ifndef MODE_DEBUG_NO_FC
    wiringPiSPIDataRW(0, bc, 1); // Send the channel, recieve the value
#endif
    nanosleep(t100000n, NULL);

    if (*bc == (val ^ 0xff) || flg1)
    {
        printf(" [correct val %d %d, %d] ", *bc, (val ^ 0xff), flg1);
        *tmp1 = val;
        *t1 = (*bc ^ 0xff);
        flg1 = 1;
    }
    else
    {
        printf(" [false val %d %d, %d] ", *bc, (val ^ 0xff), flg1);
        *tmp1 = 0xff;
        flg1 = 0;
    }

#ifndef MODE_DEBUG_NO_FC
    wiringPiSPIDataRW(0, tmp1, 1); // Send confirmation of reception, recieve channel
#endif
    nanosleep(t100000n, NULL);

    if (*tmp1 == (channel ^ 0xff) || flg2)
    {
        printf(" [correct channel %d %d, %d] ", *tmp1, (channel ^ 0xff), flg2);
        *tmp2 = channel;
        *t2 = (*tmp1 ^ 0xff);
        flg2 = 1;
    }
    else
    {
        printf(" [false channel %d %d, %d] ", *tmp1, (channel ^ 0xff), flg2);
        *tmp2 = 0xff;
        flg2 = 0;
    }

#ifndef MODE_DEBUG_NO_FC
    wiringPiSPIDataRW(0, tmp2, 1); // Send confirmation of reception, recieve something
#endif
    nanosleep(t100000n, NULL);

    /*
            If the value was wrong but channel was right, send the value only;
            and vice versa
        */
    if (flg1 && flg2)
    {
        printf("\tSuccess!, %d, %d", *t1, *t2);
        delete bv;
        delete bc;
        delete tmp1;
        delete tmp2;
        mtx.unlock();
        return;
    }
    else
    {
        printf("\t[Retrying]");
        goto back;
    }
}
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For I2C Based Communication------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined I2C_PROTOCOL
#include "LowLevel/I2Cdrivers.h"

int IssueCommand()
{
}

void Raw_Init(int argc, char *argv[])
{
}

void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For Nrf24L01 Communication------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined NRF24L01_SPI_PROTOCOL

#include "RadioDrivers.cpp"

int IssueCommand()
{
    NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void Raw_Init(int argc, char *argv[])
{
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------For MSP Based Serial directly to FC-------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MSP_Serial_PROTOCOL

#include "LowLevel/MSP/inc/msp/MSP.hpp"
#include "LowLevel/MSP/inc/msp/msg_print.hpp"
#include "LowLevel/MSP/inc/msp/msp_id.hpp"
#include "LowLevel/MSP/inc/msp/FlightController.hpp"

fcu::FlightController *FlController;

int IssueCommand()
{
    return 0;
}

void Channel_Updater(int threadId);

void Raw_Init(int argc, char *argv[])
{
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
    /*
        msp::MSP msp(device, baudrate);
        msp.setWait(1);
    */
    cout << "\n\tAttempting to connect to the Flight Controller...\n";
    std::chrono::high_resolution_clock::time_point start, end;
    bool feature_changed = false;
start:
    FlController = new fcu::FlightController(device, baudrate);

    // wait until connection is established
    // get unique box IDs
    start = std::chrono::high_resolution_clock::now();
    FlController->initialise();
    end = std::chrono::high_resolution_clock::now();
    std::cout << "FC connected, ready after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    // on cleanflight, we need to enable the "RX_MSP" feature
    if (FlController->isFirmwareCleanflight())
    {
        std::cout << "\n\n\tCleanFlight/BetaFlight FC Identified and Successfully Connected\n\n";
        if (FlController->enableRxMSP() == 1)
        {
            std::cout << "RX_MSP enabled, restart" << std::endl;
            feature_changed = true;
            goto start;
        }

        if (feature_changed)
        {
            // if we rebooted after updating the RX_MSP feature, we need to sleep for a while
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
    else if (FlController->isFirmwareMultiWii())
    {
        std::cout << "\n\n\tMultiWii FC Identified and Successfully Connected\n\n";
    }

    std::cout << "Armed? " << FlController->isArmed() << std::endl;

    // disarm the FC
    std::cout << "Disarming..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    FlController->disarm_block();
    end = std::chrono::high_resolution_clock::now();

    if (!FlController->isArmed())
    {
        std::cout << "disarmed after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
    }

    // try connecting until first package is received
    /*
        {
            std::cout << "Waiting for flight controller to become ready..." << std::endl;
            auto start = std::chrono::steady_clock::now();
            msp::msg::Ident ident;
            if (msp.request_wait(ident, 10))
            {
                auto end = std::chrono::steady_clock::now();
                std::cout << "MSP version " << (int)ident.version << " ready after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
                std::cout << "\n\n\tMultiWii FC Identified and Successfully Connected\n\n";
                // test update rate for reading Gyro messages
                {
                    const unsigned int max_msg = 1000;
                    unsigned int n_msg = 0;
                    auto start = std::chrono::steady_clock::now();
                    while (n_msg != max_msg)
                    {
                        msp::msg::ImuRaw status;
                        msp.request_block(status);
                        n_msg++;
                    }
                    auto end = std::chrono::steady_clock::now();

                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                    std::cout << "read " << max_msg << " messages in: " << duration << " ms" << std::endl;
                    std::cout << "messages per second: " << max_msg / (duration / 1000.0) << " Hz" << std::endl;
                }
            }
            else
            {
                std::cout << "error getting MSP version" << std::endl;
                exit(0);
            }
        }*/
}

uint16_t rcExpand(uint8_t val) // Basically map val from 0, 255 to 1000 to 2000
{
    uint16_t aa = 1000 + (4 * uint16_t(val)); // The formula to map ranges
    if (aa < 1000)
        aa = 1000;
    else if (aa > 2000)
        aa = 2000;

    return aa;
}

void Channel_Updater(int threadId)
{
    // Basicaly, make sure to update every 5 seconds to convey the FC that everything is fine.
    while (1)
    {
        mtx.lock();
        FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE])); //, pp->aux1, pp->aux2, 1000, 1000);
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

void sendCommand(uint8_t val, uint8_t channel)
{
    FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE])); //, pp->aux1, pp->aux2, 1000, 1000);
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------For AirSim Simple FC, through provided APIs---------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */


#if defined MODE_AIRSIM

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"


int IssueCommand(int threadId)
{
    //NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void Channel_Updater(int threadid)
{
    while(1);
}

void Raw_Init(int argc, char *argv[])
{
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif


/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------For Testing without FC, on development PC----------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */


#if defined MODE_DEBUG_NO_FC


int IssueCommand()
{
    return 0;
}

void Channel_Updater(int threadid)
{
    while(1);
}

void Raw_Init(int argc, char *argv[])
{
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------RX Definitions ends here------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------MSP Data stream for Multiwii/Betaflight FC----------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MSP_SERIAL_CLI_MONITOR)

void Channel_ViewRefresh(int threadId)
{
    while (1)
    {
#if defined(SHOW_STATUS_ARMED)
        if (FlController->isArmed())
        {
            std::cout << "Armed\t"; // after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
        }
        else
        {
            std::cout << "Disarmed\t";
        }
#endif
#if defined(SHOW_STATUS_RC)
        msp::msg::Rc rc;
        FlController->client.request(rc);
        cout << rc;
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

#endif
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ----------------------------------------------General APIs for Control-------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

void setThrottle(int throttle)
{
    unsigned char t = (unsigned char)throttle;
    mtx.lock();
    RC_DATA[THROTTLE] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(throttle, 0);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    mtx.lock();
    RC_DATA[PITCH] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(pitch, 1);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    mtx.lock();
    RC_DATA[ROLL] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(roll, 2);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    mtx.lock();
    RC_DATA[YAW] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(yaw, 3);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux1(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX1] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 5);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux2(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX2] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 5);
#endif
    mtx.unlock();
    //IssueCommand();
}
/*
ResponsePackets *getResponse()
{
    return &rff;
}*/

int BasicControls_init(int argc, char *argv[])
{
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    Raw_Init(argc, argv);
    /*
    pp->magic = CP_MAGIC;
    pp->throttle = 0;
    pp->pitch = 127;
    pp->roll = 127;
    pp->yaw = 254;*/

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

#if defined(MSP_SERIAL_CLI_MONITOR)
    thread *chnl_refresh = new thread(Channel_ViewRefresh, 0);
#endif
#if defined(UPDATER_THREAD)
    thread *chnl_update = new thread(Channel_Updater, 1);
#endif

    return 0;
}
