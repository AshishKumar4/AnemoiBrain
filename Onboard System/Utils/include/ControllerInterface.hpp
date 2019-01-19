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
#define SHOW_STATUS_ARMED
#define RC_VIEW_UPDATE_RATE 100 // Miliseconds

#if defined(MODE_AIRSIM)

#define AIRSIM_MODE_API
//#define AIRSIM_MODE_SOCKETS

#endif

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
//#define MSP_SERIAL_FORWARDING
//#define MSP_REMOTE_TWEAKS
/*
        Telemetry Type
    */
//#define NRF24
//#define WIFI
//#define Xbee

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

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX1 4
#define AUX2 5

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
int ControllerInterface_init(int argc, char **argv);
} // namespace ControllerInterface
