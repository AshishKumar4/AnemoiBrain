#pragma once

#include "stdint.h"
#include "string"
#include "iostream"
#include "vector"
#include "algorithm"
#include "mutex"
#include "thread"

#include "common.hpp"

#include "Sensors/Sensors.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "Sensors/Location.hpp"

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

#define MODE_REALDRONE

#define SYNCD_TRANSFER
#define UPDATER_THREAD

/*
    Outputs to be shown on CLI
*/

#define UPDATE_STATUS_RC
#define UPDATE_STATUS_PID
#define UPDATE_STATUS_IMU
#define UPDATE_STATUS_WIFI_STRENGTH

#define SHOW_STATUS_RC
#define SHOW_STATUS_PID
#define SHOW_STATUS_IMU
#define SHOW_POSITION
#define SHOW_VELOCITY
#define SHOW_STATUS_ARMED
#define SHOW_STATUS_WIFI_STRENGTH

//#define ACTUATION_INTENTION_RELATIVE

#define CLI_UPDATE_RATE 100 // Miliseconds
#define FAILSAFE_LANDING_RATE 40

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
    Telemetry Type
*/
//#define NRF24
//#define WIFI
//#define Xbee

/*
    Data Gathering method
*/

#define CLI_MONITOR

#if defined(MSP_Serial_PROTOCOL)
//#define MSP_SERIAL_FORWARDING
//#define MSP_REMOTE_TWEAKS

#endif

#if defined(MODE_AIRSIM)
#define AIRSIM_MODE_API
#define AUTONOMOUS_ACTUATION_CONTROLLERS
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

#define THROTTLE 0
#define PITCH 1
#define ROLL 2
#define YAW 3
#define AUX1 4
#define AUX2 5
#define AUX3 6
#define AUX4 7

#define RC_X_MOTION 8
#define RC_Y_MOTION 9

#define HEADING_YAW_P 0.8
#define HEADING_YAW_I 1
#define HEADING_YAW_D 200
#define HEADING_YAW_DAMPING 1
#define HEADING_YAW_DAMPING_2 0.8

typedef int (*func_i_t)(int);						// function pointer
typedef int (*func_vs_t)(std::vector<std::string>); // function pointer

namespace ControllerInterface
{
InertialMeasurement_t *MainIMU;
GlobalLocator_t *MainLocator;
GlobalState_t *MainState;

uint8_t RC_MASTER_DATA[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

void setDistance(float val);

uint8_t getGyro(int axis);
uint8_t getAcc(int axis);
uint8_t getMag(int axis);
uint8_t getPID_P(int axis);
uint8_t getPID_I(int axis);
uint8_t getPID_D(int axis);
uint8_t getArmStatus(int block);

float getYaw();   // Gives in Radians, Absolute for drone wrt world
float getRoll();  // Gives in Radians, Absolute for drone wrt world
float getPitch(); // Gives in Radians, Absolute for drone wrt world

float getYawDegrees();
float getRollDegrees();
float getPitchDegrees();
float getHeadingDegrees(); // Gives in Degrees
float getHeading();

float get_X_Coordinate();
float get_Y_Coordinate();
float getAltitude();

float get_X_VelocityRel();
float get_Y_VelocityRel();
float get_Z_VelocityRel();
float get_X_VelocityAbs();
float get_Y_VelocityAbs();
float get_Z_VelocityAbs();

vector3D_t getVelocityAbs();
vector3D_t getVelocityRel();

GeoPoint_t getLocation(); // CHANGE THIS
vector3D_t getVelocity();
image_t getCameraView();
data_imu_t getIMU();
DroneState_t getCompleteState();

quaternion_t getOrientationQuaternion();
vector3D_t getOrientation(); // Returns Euler angle orientation

float getDesiredVelocity();
float getCurrentTargetDistance();
void HeadlessMoveTowardsTarget(float val);

int setAutoYaw(float heading);
int setAutoRoll(float heading);
int setAutoPitch(float heading);

int setHeading(float heading);
int testHeading(int test);

int setVelocity(vector3D_t val);
void set_Y_Motion(int val);
void set_X_Motion(int val);
void set_X_MotionAbs(int val);
void set_Y_MotionAbs(int val);

int setDestinationX(float val);
int setDestinationY(float val);
int setAltitude(float altitude);

void takeOff(float altitude = 5);

int setVelocity(vector3D_t val);
int setPosition(GeoPoint_t val);
int set_X_Velocity(float val);
int set_Y_Velocity(float val);
/*
    High Level APIs 
*/

int setDestination(GeoPoint_t position, bool start_now = true);
int gotoLocation(float x, float y, float z);
int returnToHome();

int toggleFeedbackControllers(char type);

int launch_ActuationControllers();
void ResumeHandler();
void FaultHandler();

int ControllerInterface_init(int argc, const char *argv[]);
} // namespace ControllerInterface

volatile std::thread *chnl_refresh;
volatile std::thread *keyboard_handler;
volatile std::thread *chnl_update;