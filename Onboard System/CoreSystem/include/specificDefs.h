#pragma once 

#include "common.hpp"
#include <mutex>
#include <thread>

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------Some Pretty Definitions we may need-------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

static uint8_t RC_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//using namespace std;
static std::mutex Main_Mutex;
static std::mutex failsafe;

static std::thread *FailSafeThread;

static bool FaultManaged = false;
static bool FailSafeTrigger = false;

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

static timespec *t100n = new timespec;
static timespec *t1000n = new timespec;
static timespec *t10000n = new timespec;
static timespec *t100000n = new timespec;

static uint8_t IMU_Raw[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
static uint8_t PID_Raw[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
static uint8_t Current_PID_Var = 0;

static std::atomic<bool> IntentionOverride;

