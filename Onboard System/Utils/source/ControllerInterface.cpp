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

#include "ControllerInterface.hpp"

#if defined(MODE_AIRSIM)
#undef MODE_DEBUG_NO_FC
#undef FAKE_PROTOCOL

#undef MSP_SERIAL_CLI_MONITOR // We don't use MSP with airsim; instead we may use mavlink
#undef MSP_Serial_PROTOCOL
#endif

#if !defined(MODE_AIRSIM) && !defined(MODE_MAVLINK_SIM) && !defined(MODE_DEBUG_NO_FC)
#define MODE_REALDRONE
#endif

#if !defined(ONBOARD_SPI_PROTOCOL) && !defined(NRF24L01_SPI_PROTOCOL) && !defined(I2C_PROTOCOL) && !defined(MSP_Serial_PROTOCOL) && !defined(MODE_AIRSIM)
#define FAKE_PROTOCOL
#endif

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
        std::cout << "Successfully Issued Command\n";
        return 0;
    }
    return 1;
}

void Channel_Updater(int threadid)
{
    std::cout << "\nSPI Updater Initialized...";
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
            std::cout << "Couldn't Issue the command\n";
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

#include "LowLevel/msp/MSP.hpp"
#include "LowLevel/msp/msg_print.hpp"
#include "LowLevel/msp/msp_id.hpp"
#include "LowLevel/msp/FlightController.hpp"

fcu::FlightController *FlController;

int IssueCommand()
{
    return 0;
}

uint16_t rcExpand(uint8_t val) // Basically map val from 0, 255 to 1000 to 2000
{
    uint16_t aa = 1000 + uint16_t(int(3.938 * double(val))); // The formula to map ranges
    if (aa > 2000)
        aa = 2000;

    return aa;
}

void Channel_Updater(int threadId)
{
    // Basicaly, make sure to update every 5 seconds to convey the FC that everything is fine.
    while (1)
    {
        mtx.lock();
        FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), 1000, 1000);
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Raw_Init(int argc, char *argv[])
{
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
    /*
        msp::MSP msp(device, baudrate);
        msp.setWait(1);
    */
    std::cout << "\n\tAttempting to connect to the Flight Controller...\n";
    std::chrono::high_resolution_clock::time_point start, end;
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
}

void sendCommand(uint8_t val, uint8_t channel)
{
    FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), 1000, 1000);
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------For AirSim Simple FC, through provided APIs---------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#if defined(AIRSIM_MODE_SOCKET)

#elif defined(AIRSIM_MODE_API)
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

#define TIMESLICE 0.001

msr::airlib::MultirotorRpcLibClient client;

/*

def valMap(i, imin, imax, omin, omax):
    aa = omin + (((omax - omin) / (imax - imin)) * (i - imin))  # The formula to map ranges
    return aa
*/

float rcShrink(uint8_t val, float omin = -1.0, float omax = 1.0)
{
    float aa = omin + (((omax - omin) / (255.0)) * int(val));
    return aa;
}

int IssueCommand(int threadId)
{
    //NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
    return 0;
}

void Channel_Updater(int threadid)
{
    while (1)
    {
        //mtx.lock();
        client.moveByAngleThrottleAsync(rcShrink(255 - RC_DATA[PITCH]), rcShrink(RC_DATA[ROLL]), rcShrink(RC_DATA[THROTTLE], 0, 10), rcShrink(RC_DATA[YAW], -6.0, 6.0), TIMESLICE);
        //mtx.unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(int(TIMESLICE * 1000 * 1000)));
    }
}

void Raw_Init(int argc, char *argv[])
{
    //rpc::server srv(8080);

    //cout << "Press Enter to enable API control" << endl; cin.get();
    client.enableApiControl(true);

    //cout << "Press Enter to arm the drone" << endl; cin.get();
    client.armDisarm(true);

    //cout << "Press Enter to takeoff" << endl; cin.get();
    client.takeoffAsync(5); //*/
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    //client.moveByAngleThrottleAsync(rcShrink(RC_DATA[PITCH]), rcShrink(RC_DATA[ROLL]), rcShrink(RC_DATA[THROTTLE], 0, 10), rcShrink(RC_DATA[YAW], -6, 6), 10);
}

#endif
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------For Testing without FC, on development PC----------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MODE_DEBUG_NO_FC) || defined(FAKE_PROTOCOL)

int IssueCommand()
{
    return 0;
}

void Channel_Updater(int threadid)
{
    while (1)
        ;
}

void Raw_Init(int argc, char *argv[])
{
}

static volatile void sendCommand(uint8_t val, uint8_t channel)
{
    printf("\n[%d ------> %d", val, channel);
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
        mtx.lock();
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
        std::cout << rc;
#endif
#if defined(SHOW_STATUS_IMU)

#endif
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(RC_VIEW_UPDATE_RATE));
    }
}

#endif

#if defined(MSP_REMOTE_TWEAKS)

int MSP_SetPID(char *raw_data)
{
    return 0;
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------MSP Data stream forwarding------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MSP_SERIAL_FORWARDING)

//#include "LowLevel/Serial.cpp"

#include "LowLevel/msp/MSP.hpp"
#include "LowLevel/msp/msg_print.hpp"
#include "LowLevel/msp/msp_id.hpp"
#include "LowLevel/msp/FlightController.hpp"

char *bb = new char[4096];

msp::MSP *msp_agent;

MSP_Packet MSP_Agent(char *buf, int size)
{
    try
    {
        int j = 0;
        std::vector<uint8_t> vec((uint8_t *)buf, (uint8_t *)(buf + size));
        /*for (int i = 0; i < size; i++)
        {
            if (buf[i] == '$' && buf[i + 1] == 'M') // MSP Header
            {
                //buf[i+2] contains the direction
                //buf[i+3] contains the size
            }
            printf("[%x]", vec[i]);
        } //*/
        printf("\n Sending...");
        while (msp_agent->write(vec) != 1)
            ;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (msp_agent->hasData() < 1)
        {
            exit(0);
            throw "No Data";
        }

        while (char(msp_agent->read()) != '$')
            ;

        printf("\n Got Data!");

        uint8_t *rdat = (uint8_t *)bb; //malloc(1024); //sz + 1);
        rdat[0] = '$';
        rdat[1] = (uint8_t)msp_agent->read(); //hdr;
        rdat[2] = (uint8_t)msp_agent->read(); //com_state;
        rdat[3] = (uint8_t)msp_agent->read(); //data_size;
        rdat[4] = (uint8_t)msp_agent->read(); //id;

        int i;
        int sz = rdat[3] + 6;
        for (i = 5; i < rdat[3] + 5; i++)
        {
            rdat[i] = (uint8_t)msp_agent->read(); //data[i - 5];
            //printf("{{%d}}", rdat[i]);
        }

        rdat[sz - 1] = (uint8_t)msp_agent->read(); //rcv_crc;
        printf("Size: %d", sz);

        /*for (int j = 0; j < sz; j++)
        {
            printf("<<%x>>", rdat[j]);
        }*/

        return MSP_Packet((char *)rdat, sz);
    }
    catch (std::exception &e)
    {
        printf("\n Some Error");
        //while(1);
        return MSP_Packet(NULL, 0);
    }
}

//std::vector<SerialPort *> defaultPorts;

void Port_Forwarding_Init(int argc, char *argv[])
{
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
    std::cout << "Opening Port " << device << std::endl;
    msp_agent = new msp::MSP(device, baudrate);
    //defaultPorts.push_back(new SerialPort(device.c_str(), baudrate));
    /*char* buf = new char[4096];
    for(int i = 0; i < 1000; i++)
    {
       // printf(".");
        std::cout<<ReadFromPort(0, buf, 100)<<std::endl;
        printf(buf);
    }
    while(1);*/
}
/*
std::mutex serial_mtx;

int WriteToPort(int portnum, char *buff, int size)
{
    //serial_mtx.lock();
    int a = defaultPorts[portnum]->Write(buff, size);
    //printf("..");
    //serial_mtx.unlock();
    return a;
}

int ReadFromPort(int portnum, char *buff, int size)
{
    //serial_mtx.lock();
    int a = defaultPorts[portnum]->Read(buff, size);
    //serial_mtx.unlock();
    return a;
}*/

#endif

namespace ControllerInterface
{
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

int ControllerInterface_init(int argc, char **argv)
{
    printf("\n Initializing Flight Controller Interface...");
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    Raw_Init(argc, argv);
#if defined(MSP_SERIAL_FORWARDING)
    Port_Forwarding_Init(argc, argv);
#endif
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
    std::thread *chnl_refresh = new std::thread(Channel_ViewRefresh, 0);
#endif
#if defined(UPDATER_THREAD)
    std::thread *chnl_update = new std::thread(Channel_Updater, 1);
#endif

    return 0;
}

} // namespace ControllerInterface
