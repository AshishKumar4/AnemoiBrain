
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
#define AUTONOMOUS_ACTUATION_CONTROLLERS

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
#if !defined(MODE_DEBUG_NO_FC) && defined(MODE_REALDRONE)
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

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ----------------------------------------------Some Preprocessor Checkups------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MODE_AIRSIM)

#undef MSP_SERIAL_CLI_MONITOR // We don't use MSP with airsim; instead we may use mavlink
#undef MSP_Serial_PROTOCOL
#undef MODE_REALDRONE

#define AIRSIM_MODE_API
//#define AIRSIM_MODE_SOCKETS
#endif

#if !defined(MODE_AIRSIM) && !defined(MODE_MAVLINK_SIM) && !defined(MODE_DEBUG_NO_FC)
#define MODE_REALDRONE
#endif

#if defined(MODE_DEBUG_FC) || (!defined(ONBOARD_SPI_PROTOCOL) && !defined(NRF24L01_SPI_PROTOCOL) && !defined(I2C_PROTOCOL) && !defined(MSP_Serial_PROTOCOL) && !defined(MODE_AIRSIM))
#define FAKE_PROTOCOL
#endif
