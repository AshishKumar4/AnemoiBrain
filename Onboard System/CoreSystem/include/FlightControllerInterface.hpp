#pragma once

#include "common.hpp"
#include <mutex>
#include <thread>

typedef int (*func_i_t)(int);						// function pointer
typedef int (*func_vs_t)(std::vector<std::string>); // function pointer

class FlightController
{
	std::string name;
	std::string firmware;
	std::string variant;

	uintptr_t desc;

public:
	FlightController(std::string name, std::string firmware, std::string variant, uintptr_t desc) : name(name), firmware(firmware), variant(variant), desc(desc)
	{
	}

	~FlightController() 
	{
	}

	uintptr_t getDesc()
	{
		return desc;
	}
};

int IssueCommand();
void Channel_Updater(int threadId);
void Raw_Init(int argc, const char *argv[]);
void sendCommand(uint8_t val, uint8_t channel);

namespace ControllerInterface
{
FlightController *MainFC;
}

void destroyFlightControllerObjs();


/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------Some Pretty Definitions we may need-------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

//using namespace std;
std::mutex Main_Mutex;
std::mutex failsafe;

uint8_t RC_MASTER_DATA[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t RC_DATA[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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

bool IntentionOverride;

