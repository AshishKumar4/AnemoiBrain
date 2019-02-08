#pragma once

#include "stdint.h"
#include "string"
#include "iostream"
#include "vector"
#include "algorithm"
#include "mutex"
#include "thread"

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

//#define ACTUATION_INTENTION_RELATIVE

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
#define SHOW_STATUS_ARMED
#define SHOW_STATUS_WIFI_STRENGTH

#define CLI_UPDATE_RATE 100 // Miliseconds
#define FAILSAFE_LANDING_RATE 10

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
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define RC_X_MOTION 8
#define RC_Y_MOTION 9

#define HEADING_YAW_P 0.8
#define HEADING_YAW_I 1
#define HEADING_YAW_D 200
#define HEADING_YAW_DAMPING 1
#define HEADING_YAW_DAMPING_2 0.8

class vector3D_t
{
public:
    float x;
    float y;
    float z;

    vector3D_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    
    vector3D_t(float xval, float yval, float zval)
    {
        x = xval;
        y = yval; 
        z = zval;
    }
};

class quaternion_t 
{
    float _w;
    float _x;
    float _y;
    float _z;
public: 

    quaternion_t()
    {
        _w = 1;
        _x = 0;
        _y = 0;
        _z = 0;
    }

    quaternion_t(float w, float x, float y, float z)
    {
        this->_w = w;
        this->_x = x;
        this->_y = y;
        this->_z = z;
    }

    float w()
    {
        return _w;
    }

    float x()
    {
        return _x;
    }

    float y()
    {
        return _y;
    }

    float z()
    {
        return _z;
    }
};

typedef int (*func_i_t)(int);              // function pointer
typedef int (*func_vs_t)(std::vector<std::string>);              // function pointer

namespace ControllerInterface
{
uint8_t RC_MASTER_DATA[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

quaternion_t getOrientationQuaternion();
vector3D_t getOrientation(); // Returns Euler angle orientation
float getYaw(); // Gives in Radians
float getRoll(); // Gives in Radians
float getPitch(); // Gives in Radians

float getYawDegrees();
float getRollDegrees();
float getPitchDegrees();
float getHeadingDegrees(); // Gives in Degrees
float getHeading();

float get_X_Coordinate();
float get_Y_Coordinate();
float getAltitude();

int setAutoYaw(float heading);
int setAutoRoll(float heading);
int setAutoPitch(float heading);

int setHeading(float heading);
int testHeading(int test);
void setAlititude(float altitude);
void takeOff(float altitude = 5);
vector3D_t getVelocity();
vector3D_t getPosition();
/*
    High Level APIs 
*/

int autoNavPID(vector3D_t start, vector3D_t destination, float maxAltitude = 0);
int setDestination(vector3D_t position, bool start_now = true);
int returnToHome();

//void Remote_API_Invoker(int code, std::vector<std::string> args);
void RemoteAPI_Invoker(int code, int count);
void ResumeHandler();
void FaultHandler();

int ControllerInterface_init(int argc, char **argv);
} // namespace ControllerInterface
