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
#include <future>

#include "ControllerInterface.hpp"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------Some Pretty Definitions we may need-------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

uint8_t RC_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//using namespace std;
std::mutex mtx;
std::mutex failsafe;

std::thread *FailSafeThread;

bool FaultManaged = false;
bool FailSafeTrigger = false;

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

uint8_t IMU_Raw[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t PID_Raw[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t Current_PID_Var = 0;

struct MSP_Packet
{
    char *buf;
    int size;

    MSP_Packet(char *buf, int sz) : buf(buf), size(sz){};
};

MSP_Packet MSP_Agent(char *buf, int size);

func_vs_t API_ProcedureInvokeTable[256];

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ----------------------------------------------Some Preprocessor Checkups------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MODE_AIRSIM)
#undef MSP_SERIAL_CLI_MONITOR // We don't use MSP with airsim; instead we may use mavlink
#undef MSP_Serial_PROTOCOL
#endif

#if !defined(MODE_AIRSIM) && !defined(MODE_MAVLINK_SIM) && !defined(MODE_DEBUG_NO_FC)
#define MODE_REALDRONE
#endif

#if defined(MODE_DEBUG_FC) || (!defined(ONBOARD_SPI_PROTOCOL) && !defined(NRF24L01_SPI_PROTOCOL) && !defined(I2C_PROTOCOL) && !defined(MSP_Serial_PROTOCOL) && !defined(MODE_AIRSIM))
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
        try
        {
            mtx.lock();
            FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), rcExpand(RC_DATA[AUX3]), rcExpand(RC_DATA[AUX4]));
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (std::exception &e)
        {
            std::cout << "Error in CLI Monitor " << e.what();
            mtx.unlock();
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            mtx.unlock();
        }
    }
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
    FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), rcExpand(RC_DATA[AUX3]), rcExpand(RC_DATA[AUX4]));
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
        try
        { 
            //mtx.lock();
            client.moveByAngleThrottleAsync(rcShrink(255 - RC_DATA[PITCH]), rcShrink(RC_DATA[ROLL]), rcShrink(RC_DATA[THROTTLE], 0, 5), rcShrink(RC_DATA[YAW], -6.0, 6.0), TIMESLICE);
            //mtx.unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(int(TIMESLICE * 1000.0 * 1000.0)));
        }
        catch (std::exception &e)
        {
            std::cout << "Error in CLI Monitor " << e.what();
            //mtx.unlock();
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            //mtx.unlock();
        }
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
/* ------------------------------------------Protocol Specific Data Gathering---------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MSP_Serial_PROTOCOL)
#endif

#if defined(MODE_AIRSIM)
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------Cli Monitor------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(CLI_MONITOR)

int show_RC = 0, show_PID = 0, show_IMU = 0, show_Wifi = 0, show_armed = 1;

void Channel_ViewRefresh(int threadId)
{
    while (1)
    {
        try
        {
            mtx.lock();
#if defined(MSP_Serial_PROTOCOL)
            if (show_armed)
            {
                if (FlController->isArmed())
                {
                    std::cout << "Armed\t"; // after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
                }
                else
                {
                    std::cout << "Disarmed\t";
                }
            }
#if defined(UPDATE_STATUS_RC)
            if (show_RC)
            {
                msp::msg::Rc rc;
                FlController->client.request(rc);
#if defined(SHOW_STATUS_RC)
                std::cout << rc;
#endif
            }
#endif
#if defined(UPDATE_STATUS_IMU)
            if (show_IMU)
            {
                msp::msg::ImuRaw imu;
                FlController->client.request(imu);
                for (int i = 0; i < 3; i++)
                    IMU_Raw[0][i] = (uint8_t)imu.gyro[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[1][i] = (uint8_t)imu.acc[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[2][i] = (uint8_t)imu.magn[i];
#if defined(SHOW_STATUS_IMU)
                std::cout << imu;
#endif
            }
#endif
#if defined(UPDATE_STATUS_PID)
            if (show_PID)
            {
                msp::msg::Pid pid;
                FlController->client.request(pid);
#if defined(SHOW_STATUS_PID)
                std::cout << pid;
#endif
                PID_Raw[0][0] = (uint8_t)pid.roll.P;
                PID_Raw[1][0] = (uint8_t)pid.roll.I;
                PID_Raw[2][0] = (uint8_t)pid.roll.D;
                PID_Raw[0][1] = (uint8_t)pid.pitch.P;
                PID_Raw[1][1] = (uint8_t)pid.pitch.I;
                PID_Raw[2][1] = (uint8_t)pid.pitch.D;
                PID_Raw[0][2] = (uint8_t)pid.yaw.P;
                PID_Raw[1][2] = (uint8_t)pid.yaw.I;
                PID_Raw[2][2] = (uint8_t)pid.yaw.D;
            }
#endif
#endif // MSP
/*
    For AisSim
*/
#if defined(MODE_AIRSIM)
#if defined(UPDATE_STATUS_RC)
            if (show_RC)
            {
#if defined(SHOW_STATUS_RC)
                printf("\n[%d]-[%d]-[%d]-[%d]", RC_DATA[PITCH], RC_DATA[ROLL], RC_DATA[THROTTLE], RC_DATA[YAW]);
#endif
            }
#endif
#if defined(UPDATE_STATUS_IMU)
            if (show_IMU)
            {
#if defined(SHOW_STATUS_IMU)
                printf("\nHeading: %f", ControllerInterface::getHeading());
#endif
            }
#endif
#endif
/*
    General
*/
#if defined(UPDATE_STATUS_WIFI_STRENGTH)
#if defined(SHOW_STATUS_WIFI_STRENGTH)
            if (show_Wifi)
            {
                system("awk 'NR==3 {print \"WiFi Signal Strength = \" \$3 \"00 %\"}''' /proc/net/wireless");
            }
#endif
#endif
            fflush(stdout);
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(CLI_UPDATE_RATE));
        }
        catch (std::exception &e)
        {
            std::cout << "Error in CLI Monitor " << e.what();
            mtx.unlock();
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            mtx.unlock();
        }
    }
}

int event_key_A()
{
    if (!show_armed)
        show_armed = 1;
    else
        show_armed = 0;
    return 0;
}

int event_key_r()
{
    if (!show_Wifi)
        show_Wifi = 1;
    else
        show_Wifi = 0;
    return 0;
}

int event_key_q()
{
    if (!show_RC)
        show_RC = 1;
    else
        show_RC = 0;
    return 0;
}

int event_key_w()
{
    if (!show_PID)
        show_PID = 1;
    else
        show_PID = 0;
    return 0;
}

int event_key_e()
{
    if (!show_IMU)
        show_IMU = 1;
    else
        show_IMU = 0;
    return 0;
}

int event_key_other()
{
    return 0;
}

typedef int (*func_t)(); // function pointer
func_t KeyMap[256];

int Keyboard_handler(int id)
{
    while (1)
    {
        try
        {
            char key = getc(stdin);
            //printf("\t\t>>> [%d] <<<", (int)key);
            KeyMap[int(key)]();
        }
        catch (std::exception &e)
        {
            std::cout << e.what();
            continue;
        }
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

/*
    General Purpose tools
*/

float clamp(float val, float imin, float imax, float omin, float omax)
{
    float aa = omin + (((omax - omin) / (imax - imin)) * (val - imin));
    return aa;
}

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
    // AUX1 to be used for PID Tuning, should set which
    RC_DATA[AUX1] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 4);
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

void setAux3(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX3] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 6);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux4(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX4] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 7);
#endif
    mtx.unlock();
    //IssueCommand();
}

uint8_t getGyro(int axis)
{
    return IMU_Raw[0][axis];
}

uint8_t getAcc(int axis)
{
    return IMU_Raw[1][axis];
}

uint8_t getMag(int axis)
{
    return IMU_Raw[2][axis];
}

uint8_t getPID_P(int axis)
{
    return PID_Raw[0][axis];
}

uint8_t getPID_I(int axis)
{
    return PID_Raw[1][axis];
}

uint8_t getPID_D(int axis)
{
    return PID_Raw[2][axis];
}

uint8_t getArmStatus(int block)
{
    return 0;
}

/*
    GPS/Barometer/Compass assisted
    All SI Units, Meters, Degrees
*/

namespace // Anonymous Namespace
{
vector3D_t *controlledPosition;
vector3D_t *controlledOrientation;

std::thread *YawControllerThread;
std::thread *RollControllerThread;
std::thread *PitchControllerThread;
std::thread *AltitudeControllerThread;

std::mutex YawControllerlock;
std::mutex PitchControllerlock;
std::mutex RollControllerlock;
std::mutex AltitudeControllerlock;

float Controlled_IntendedYawHeading = 0;
float Controlled_IntendedPitchHeading = 0;
float Controlled_IntendedRollHeading = 0;
float Controlled_IntendedAltitude = 0;

void YawController()
{
    /*
        Continous Feedback loop for maintaining a constant orientation and altitude
    */
    /*
        A Simple Control Loop
    */
    try
    {
        float h = 1;
        float errorScale = HEADING_YAW_P;
        float yawVal = 127;
        float currentYaw = 127;
        float oldError = 1;
        float oldAbsError = 1;
        float deltaTime = 5;
        while (true)
        {
            try
            {
                YawControllerlock.lock();
                h = (getHeadingDegrees());
                float newError = h - Controlled_IntendedYawHeading;
                if (newError > 180 || newError < -180)
                {
                    newError -= 360;
                }
                float absNewError = std::abs(newError);
                if (absNewError <= 2)
                {
                    //direc *= 0.99;
                    setYaw(127);                                               // Make it not move anymore
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for it to stabilize
                    if (std::abs(getHeadingDegrees() - h) < 2)
                        printf("\nDone...%f %f", newError, h);
                    YawControllerlock.unlock();
                    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                    //return 1;
                }
                // Our Equation
                float clampedVal = newError * errorScale;                                                            //clamp(newError * errorScale, -180, 180, -127, 127);
                yawVal = ((clampedVal) + ((newError - oldError) / deltaTime) * HEADING_YAW_D) + RC_MASTER_DATA[YAW]; //currentYaw; //(newError/error); // - (newError / oldError)*(absNewError/newError)*2.0 // 2*(oldAbsError - 1*absNewError)

                if (yawVal > 255)
                    yawVal = 255;
                else if (yawVal < 0)
                    yawVal = 0;

                printf("\n{Errors: <%f> %f %f %f [%f]}", h, newError, yawVal, clampedVal, errorScale);
                setYaw(int(yawVal));
                oldError = newError;
                oldAbsError = absNewError;
                YawControllerlock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(int(deltaTime)));
            }
            catch (std::exception &e)
            {
                std::cout << "Error Occured! inside " << e.what();
                YawControllerlock.unlock();
            }
            catch (const std::future_error &e)
            {
                std::cout << "Caught a future_error with code \"" << e.code()
                          << "\"\nMessage: \"" << e.what() << "\"\n";
                YawControllerlock.unlock();
            }
        }
    }
    catch (...)
    {
        std::cout << "Error Occured! outside";
    }
} // namespace

vector3D_t eulerFromQuaternion(quaternion_t orien)
{
    vector3D_t oo;
    /*
        Quaternion to Euler
    */
    //std::cout << ">>>" << orien << "<<<" << std::endl;
    double heading = 0, roll, pitch, yaw;
    double ysqr = orien.y() * orien.y();

    // roll (x-axis rotation)
    double t0 = +2.0f * (orien.w() * orien.x() + orien.y() * orien.z());
    double t1 = +1.0f - 2.0f * (orien.x() * orien.x() + ysqr);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0f * (orien.w() * orien.y() - orien.z() * orien.x());
    t2 = ((t2 > 1.0f) ? 1.0f : t2);
    t2 = ((t2 < -1.0f) ? -1.0f : t2);
    pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0f * (orien.w() * orien.z() + orien.x() * orien.y());
    double t4 = +1.0f - 2.0f * (ysqr + orien.z() * orien.z());
    yaw = std::atan2(t3, t4);
    //heading = yaw;
    //printf("->Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
    //printf("->Heading: { %f %f}", orien.z(), orien.w());
    oo.x = pitch;
    oo.y = roll;
    oo.z = yaw; // // 0, 360);
    return oo;
}

} // namespace

int init_SpacialControllers()
{
    YawControllerThread = new std::thread(YawController);
    return 0;
}

int destroy_SpacialControllers()
{
    YawControllerThread->join();
    return 0;
}

int pauseControllers()
{
    YawControllerlock.lock();
    return 1;
}

int resumeControllers()
{
    YawControllerlock.unlock();
    return 0;
}

quaternion_t getOrientationQuaternion()
{
#if defined(MODE_AIRSIM)
    auto orien = client.getMultirotorState().getOrientation();
    return quaternion_t(orien.w(), orien.x(), orien.y(), orien.z());
#else
    return quaternion_t(1, 0, 0, 0);
#endif
}

vector3D_t getOrientation() // Returns Euler angle orientation
{
    return eulerFromQuaternion(getOrientationQuaternion());
}

float getYaw() // Gives in Radians
{
    float h = getOrientation().z;
    return h;
}

float getRoll() // Gives in Radians
{
    float h = getOrientation().y;
    return h;
}

float getPitch() // Gives in Radians
{
    float h = getOrientation().x;
    return h;
}

/*
    This is our convention, counter clockwise, 0 to 360 in every direction
*/

float getConventionalDegrees(float rads)
{
    // 0 -> North
    // 90 -> east
    // 180 -> south
    // 270 ->west
    float h = clamp(rads, -3.14159265358979323846, 3.14159265358979323846, 180, -180);
    if (h < 0)
    {
        h = 360 + h;
    }
    return h;
}

float getYawDegrees()
{
    return getConventionalDegrees(getYaw());
}

float getRollDegrees()
{
    return getConventionalDegrees(getRoll());
}

float getPitchDegrees()
{
    return getConventionalDegrees(getPitch());
}

float getHeadingDegrees() // Gives in Degrees
{
    return getYawDegrees();
}

float getHeading()
{
    return getHeadingDegrees();
}

int setAutoYaw(float heading)
{
    YawControllerlock.lock();
    Controlled_IntendedYawHeading = heading;
    YawControllerlock.unlock();
    return 0;
}

int setAutoRoll(float heading)
{
    RollControllerlock.lock();
    Controlled_IntendedRollHeading = heading;
    RollControllerlock.unlock();
    return 0;
}

int setAutoPitch(float heading)
{
    PitchControllerlock.lock();
    Controlled_IntendedPitchHeading = heading;
    PitchControllerlock.unlock();
    return 0;
}

int setHeading(float heading)
{
    return setAutoYaw(heading);
}

int testHeading(std::vector<std::string> test)
{
    std::cout << "Testing Auto Heading feature...";
    return setHeading(90);
}

vector3D_t getVelocity()
{
    vector3D_t vel;
    return vel;
}

vector3D_t getPosition()
{
    vector3D_t pos;
    return pos;
}

void setAltitude(float altitude)
{
}

void takeOff(float altitude)
{
    setAltitude(5);
}

void holdPosition()
{
}

void moveToRelativePosition(float x, float y, float z)
{
}

/*
    High Level APIs 
*/

int autoNavPID(vector3D_t start, vector3D_t destination, float maxAltitude)
{
    // Firstly move to the direction to face the destination

    // Then Throttle Up to maintain an altitude.
    return 0;
}

std::vector<vector3D_t> destinationBuffer;

int setDestination(vector3D_t position, bool start_now)
{
    if (start_now)
        autoNavPID(getPosition(), position);
    return 0;
}

int returnToHome()
{
    return 0;
}

/*
    Some other Mechanisms
*/

int unusedAPIhandler(std::vector<std::string> test)
{
    std::cout << "\nOops! Seems someone sent me some wrong code!";
}

void RemoteAPI_Invoker(int code, int count)
{
    std::vector<std::string> aa;
    printf("<<%d, %d>>", code, count);
    std::cout << API_ProcedureInvokeTable[code](aa);
}

void FailSafeMechanism()
{
    try
    {
        // Decrease the throttle at constant rate to land
        std::cout << "\nInitiating FailSafe...";
        setYaw(127);
        setRoll(127);
        setPitch(127);
        setYaw(127);
        int a = RC_DATA[THROTTLE];
        for (int i = a; i > 0; i--)
        {
            failsafe.lock();
            if (FaultManaged)
            {
                failsafe.unlock();
                return;
            }
            std::cout << "\nLowering the throttle to " << i;
            setThrottle(i);
            failsafe.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(FAILSAFE_LANDING_RATE));
        }
        // Disarm and send FailSafe!
        FailSafeTrigger = true;
        setAux1(51); // Trigger Failsafe
        std::cout << "\nFailSafe Locked!";
        mtx.lock(); // Grab the lock and don't release until the fault is fixed
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 2!" << e.what();
    }
}

void ResumeHandler()
{
#if defined(MSP_Serial_PROTOCOL)
    try
    {
        failsafe.lock();
        FaultManaged = true;
        failsafe.unlock();
        FailSafeThread->join();
        if (FailSafeTrigger)
        {
            mtx.unlock(); // Grab the lock and don't release until the fault is fixed
            FailSafeTrigger = false;
        }
        std::cout << "Fault Resumed and Managed!\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 3!" << e.what();
    }
#endif
}

void FaultHandler()
{
#if defined(MSP_Serial_PROTOCOL)
    try
    {
        std::cout << "Fault Occured!\nTriggering FailSafe!!!";
        FaultManaged = false;
        FailSafeThread = new std::thread(FailSafeMechanism);
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 1!" << e.what();
    }
#endif
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

    for (int i = 0; i < 256; i++)
    {
        API_ProcedureInvokeTable[i] = unusedAPIhandler;
    }
    API_ProcedureInvokeTable[111] = testHeading;

    init_SpacialControllers();

#if defined(CLI_MONITOR)
    std::thread *chnl_refresh = new std::thread(Channel_ViewRefresh, 0);
    for (int i = 0; i < 256; i++)
        KeyMap[i] = event_key_other;
    KeyMap['q'] = event_key_q; // Show RC Values
    KeyMap['w'] = event_key_w; // show PID Values
    KeyMap['e'] = event_key_e; // show IMU Data
    KeyMap['r'] = event_key_r; // show Wifi Strength
    KeyMap['A'] = event_key_A; // show Armed
    std::thread *keyboard_handler = new std::thread(Keyboard_handler, 2);
#endif
#if defined(UPDATER_THREAD)
    std::thread *chnl_update = new std::thread(Channel_Updater, 1);
#endif

    return 0;
}

} // namespace ControllerInterface
