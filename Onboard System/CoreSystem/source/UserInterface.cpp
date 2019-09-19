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
#include <functional>
#include <math.h>

#include "FlightControllerInterface.hpp"
#include "UserInterface.hpp"
#include "ControllerInterface.hpp"


/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------Cli Monitor------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(CLI_MONITOR)

#if defined(MSP_Serial_PROTOCOL)

#include <msp/msp_msg.hpp>
#include <msp/FlightController.hpp>
#include <msp/FirmwareVariants.hpp>

extern fcu::FlightController* FlController;
extern msp::FirmwareVariant FCvariant; 

#endif

void Channel_ViewRefresh()
{
#if !defined(UNIFIED_UPDATER)
    while (1)
#endif
    {
        try
        {
			// printf("\nHERE!!");
            // fflush(stdout);
            Main_Mutex.lock();
#if defined(MSP_Serial_PROTOCOL)
            if (show_armed)
            {
				// printf("\tarm cmd");
            	// fflush(stdout);
                if (FlController->isArmed())
                {
                    std::cout << "Armed\t"; // after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
                }
                else
                {
                    std::cout << "Disarmed\t";
                }
            }
            if (show_status)
            {
				// printf("\tarm cmd");
            	// fflush(stdout);
                msp::msg::Status status(FCvariant);
                FlController->sendMessage(status, 1);
				std::cout << status;
            }
#if defined(UPDATE_STATUS_RC)
            if (show_RC)
            {
				// printf("\trc cmd");
            	// fflush(stdout);
                msp::msg::Rc rc(FCvariant);
                FlController->sendMessage(rc, 1);
#if defined(SHOW_STATUS_RC)
                std::cout << rc;
                printf("\n[%d]-[%d]-[%d]-[%d]", RC_DATA[PITCH], RC_DATA[ROLL], RC_DATA[THROTTLE], RC_DATA[YAW]);
#endif
            }
#endif
#if defined(UPDATE_STATUS_IMU)
            if (show_IMU)
            {
                msp::msg::RawImu imu(FCvariant);
                FlController->sendMessage(imu, 1);
                for (int i = 0; i < 3; i++)
                    IMU_Raw[0][i] = (uint8_t)imu.gyro[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[1][i] = (uint8_t)imu.acc[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[2][i] = (uint8_t)imu.mag[i];
#if defined(SHOW_STATUS_IMU)
                std::cout << imu;
#endif
            }
#endif
#if defined(UPDATE_STATUS_PID)
            if (show_PID)
            {
                msp::msg::Pid pid(FCvariant);
                FlController->sendMessage(pid, 1);
#if defined(SHOW_STATUS_PID)
                std::cout << pid;
#endif
                // PID_Raw[0][0] = (uint8_t)pid.roll.P;
                // PID_Raw[1][0] = (uint8_t)pid.roll.I;
                // PID_Raw[2][0] = (uint8_t)pid.roll.D;
                // PID_Raw[0][1] = (uint8_t)pid.pitch.P;
                // PID_Raw[1][1] = (uint8_t)pid.pitch.I;
                // PID_Raw[2][1] = (uint8_t)pid.pitch.D;
                // PID_Raw[0][2] = (uint8_t)pid.yaw.P;
                // PID_Raw[1][2] = (uint8_t)pid.yaw.I;
                // PID_Raw[2][2] = (uint8_t)pid.yaw.D;
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
                printf("\nHeading: %f[%f],\tPitch: %f,\tRoll %f", ControllerInterface::getYaw(), ControllerInterface::getHeadingDegrees(),ControllerInterface::getPitchDegrees(), ControllerInterface::getRollDegrees());
#endif
            }
#endif
#if defined(SHOW_POSITION)
            if (show_POSITION)
            {
                printf("\nX: %f\tY: %f\tZ: %f", ControllerInterface::get_X_Coordinate(), ControllerInterface::get_Y_Coordinate(), ControllerInterface::getAltitude());
            }
#endif

#if defined(SHOW_VELOCITY)
            if (show_VELOCITY)
            {
				//printf("\nForward Velocity[%f] ", ControllerInterface::get_Y_VelocityRel());
                printf("\nvX: %f\tvY: %f\tvZ: %f", ControllerInterface::get_X_VelocityRel(), ControllerInterface::get_Y_VelocityRel(), ControllerInterface::get_Z_VelocityRel());
                //printf("\naX: %f\taY: %f\taZ: %f", ControllerInterface::get_X_VelocityAbs(), ControllerInterface::get_Y_VelocityAbs(), ControllerInterface::get_Z_VelocityAbs());
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
			// printf("\tExiting");
            fflush(stdout);
            Main_Mutex.unlock();
#if !defined(UNIFIED_UPDATER)
			std::this_thread::sleep_for(std::chrono::milliseconds(CLI_UPDATE_RATE));
#endif
        }
        catch (const std::future_error &e)
        {
            std::cout << "<Channel_ViewRefresh>Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            // Main_Mutex.unlock();
        }
        catch (std::exception &e)
        {
            std::cout << "Error in CLI Monitor " << e.what();
            // Main_Mutex.unlock();
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

int event_key_P()
{
    if (!show_POSITION)
        show_POSITION = 1;
    else
        show_POSITION = 0;
    return 0;
}

int event_key_V()
{
    if (!show_VELOCITY)
        show_VELOCITY = 1;
    else
        show_VELOCITY = 0;
    return 0;
}

int event_key_s()
{
    if (!show_status)
        show_status = 1;
    else
        show_status = 0;
    return 0;
}

int event_key_h()
{
    float p, i, d;
    char type;
    std::cin>>type;
    std::cin>>p>>i>>d;
    printf("\t {%f, %f, %f}", p, i, d);
    // if(type == 'Y')
    // {
    //     ControllerInterface::FeedbackControl::YawActuator.CONTROLLER_P = p;
    //     ControllerInterface::FeedbackControl::YawActuator.CONTROLLER_I = i;
    //     ControllerInterface::FeedbackControl::YawActuator.CONTROLLER_D = d;
    // }

    // if(type == 'A')
    // {
    //     ControllerInterface::FeedbackControl::Z_Actuator.CONTROLLER_P = p;
    //     ControllerInterface::FeedbackControl::Z_Actuator.CONTROLLER_I = i;
    //     ControllerInterface::FeedbackControl::Z_Actuator.CONTROLLER_D = d;
    // }

    // if(type == 'H')
    // {
    //     ControllerInterface::FeedbackControl::Distance_Actuator.CONTROLLER_P = p;
    //     ControllerInterface::FeedbackControl::Distance_Actuator.CONTROLLER_I = i;
    //     ControllerInterface::FeedbackControl::Distance_Actuator.CONTROLLER_D = d;
    // }
    return 0;
}

int event_key_other()
{
    return 0;
}

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
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            continue;
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
