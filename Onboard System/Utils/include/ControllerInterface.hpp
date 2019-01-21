#pragma once

#include "stdint.h"
#include "string"
#include "iostream"
#include "vector"
#include "algorithm"
#include "mutex"

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

#define SYNCD_TRANSFER
#define UPDATER_THREAD

/*
    Outputs to be shown on CLI
*/

#define SHOW_STATUS_RC
#define SHOW_STATUS_PID
#define SHOW_STATUS_ARMED
#define RC_VIEW_UPDATE_RATE 100 // Miliseconds

/*
        There are two possible configurations, 
        1) RPI unit is on board the drone and communicates with 
            flight controller, and so the telemetry unit is connected to RPI directly.
        2) RPI unit is off board the drone and communicates with the flight controller through Radio (wifi/ Xbee)
    */

/*
        Telemetry Protocol
    */

#if !defined(MODE_DEBUG_NO_FC)
//#define ONBOARD_SPI_PROTOCOL
//#define NRF24L01_SPI_PROTOCOL
//#define I2C_PROTOCOL
#define MSP_Serial_PROTOCOL
#endif

/*
        Data Gathering method
    */

#if defined(MSP_Serial_PROTOCOL)
#define MSP_SERIAL_CLI_MONITOR
//#define MSP_SERIAL_FORWARDING
//#define MSP_REMOTE_TWEAKS
#endif
/*
        Telemetry Type
    */
//#define NRF24
//#define WIFI
//#define Xbee

#if defined(MODE_AIRSIM)

#define AIRSIM_MODE_API
//#define AIRSIM_MODE_SOCKETS

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------------Some Definitions----------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#define CP_MAGIC 110

uint8_t checksum(uint8_t *buf, int len);
#define CPACKET_MAGIC 110
#define REQ_SIGNAL 251
#define REQ2_SIGNAL 101
#define ACCEPT_SIGNAL 252
#define RPACKET_MAGIC 120
#define FALSE_PACKET 145
#define ACK_GOT_PACKET 250

#define THROTTLE    0
#define PITCH       1
#define ROLL        2
#define YAW         3
#define AUX1       1
#define AUX2       2
#define AUX3       3
#define AUX4       3

uint8_t RC_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//using namespace std;
std::mutex mtx;

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

uint8_t IMU_Raw[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t PID_Raw[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t Current_PID_Var = 0;

struct MSP_Packet
{
    char *buf;
    int size;

    MSP_Packet(char *buf, int sz) : buf(buf), size(sz){};
};

MSP_Packet MSP_Agent(char *buf, int size);

namespace ControllerInterface
{
int WriteToPort(int portnum, char *buff, int size);
int ReadFromPort(int portnum, char *buff, int size);
void setThrottle(int throttle);
void setPitch(int pitch);
void setRoll(int roll);
void setYaw(int yaw);
void setAux1(int val);
void setAux2(int val);
void setAux3(int val);
void setAux4(int val);

uint8_t getGyro(int axis);
uint8_t getAcc(int axis);
uint8_t getMag(int axis);
uint8_t getPID_P(int axis);
uint8_t getPID_I(int axis);
uint8_t getPID_D(int axis);
uint8_t getArmStatus(int block);

int ControllerInterface_init(int argc, char **argv);
} // namespace ControllerInterface
