
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

#include "Sensors/Sensors.hpp"
#include "ControllerInterface.hpp"
// #include "specificDefs.h"
#include "CommonControl.hpp"
#include "AutoNavigation.hpp"

#include "FlightControllerInterface.cpp"
#include "UserInterface.cpp"

#if defined(MODE_CONTROL_EXTERNAL)
#include "ExternalControl.cpp"
#else
#include "FeedbackControl.cpp"
#endif

namespace ControllerInterface
{

volatile std::thread *chnl_refresh;
volatile std::thread *keyboard_handler;
volatile std::thread *chnl_update;

StateEstimator_t *MainState;

uint8_t *RC_APPARENT_DATA;

void setRC_Direct(int channel, uint8_t val)
{
	RC_DATA[channel] = val;
}

void setRC_Buffered(int channel, uint8_t val)
{
	RC_MASTER_DATA[channel] = val;
}

uint8_t getRC_Direct(int channel)
{
	return RC_DATA[channel];
}

uint8_t getRC_Buffered(int channel)
{
	return RC_MASTER_DATA[channel];
}

StateEstimator_t *getMainStateEstimator()
{
	return MainState;
}
/* ------------------------------------------------------------------------------------------------------------------------ */
/* ----------------------------------------------General APIs for Control-------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/******************************************************************************************/
/******************************* A Low-High level Get APIs ********************************/
/******************************************************************************************/

uint8_t getGyro(int axis)
{
	//return IMU_Raw[0][axis];
	try
	{
		return IMU_Raw[0][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getGyro>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

uint8_t getAcc(int axis)
{
	//return IMU_Raw[1][axis];
	try
	{
		return IMU_Raw[1][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getAcc>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

uint8_t getMag(int axis)
{
	//return IMU_Raw[2][axis];
	try
	{
		return IMU_Raw[2][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getMag>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

vector3D_t getGyro()
{
	return vector3D_t(IMU_Raw[0]);
}

vector3D_t getAcc()
{
	return vector3D_t(IMU_Raw[1]);
}

vector3D_t getMag()
{
	return vector3D_t(IMU_Raw[2]);
}

uint8_t getPID_P(int axis)
{
	//return PID_Raw[0][axis];
	try
	{
		return PID_Raw[0][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getGyro>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

uint8_t getPID_I(int axis)
{
	//return PID_Raw[1][axis];
	try
	{
		return PID_Raw[1][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getPID_I>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

uint8_t getPID_D(int axis)
{
	//return PID_Raw[2][axis];
	try
	{
		return PID_Raw[2][axis];
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getPID_D>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

uint8_t getArmStatus(int block)
{
	return 0;
}

float getYaw() // Gives in Radians
{
	//   return MainState->getYaw();
	try
	{
		return MainState->getYaw();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getYaw>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getRoll() // Gives in Radians
{
	//return MainState->getRoll();
	try
	{

		return MainState->getRoll();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getRoll>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getPitch() // Gives in Radians
{
	//return MainState->getPitch();
	try
	{
		return MainState->getPitch();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getPitch>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getYawDegrees()
{
	//return MainState->getYawDegrees();
	try
	{
		return MainState->getYawDegrees();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getYawDegrees>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getRollDegrees()
{
	//return MainState->getRollDegrees();
	try
	{
		return MainState->getRollDegrees();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getRollDegrees>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getPitchDegrees()
{
	//return MainState->getPitchDegrees();
	try
	{
		return MainState->getPitchDegrees();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getPitchDegrees>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float get_X_Coordinate()
{
	//return MainState->get_X_Coordinate();
	try
	{
		return MainState->get_X_Coordinate();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<get_X_Coordinate>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float get_Y_Coordinate()
{
	//return MainState->get_Y_Coordinate();
	try
	{
		return MainState->get_Y_Coordinate();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<get_Y_Coordinate>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float get_X_VelocityRel()
{
	try
	{
		return MainState->get_X_VelocityRel();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<get_X_VelocityRel>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost get_X_VelocityRel loop!" << e.what();
	}
	return 0;
}

float get_Y_VelocityRel()
{
	try
	{
		return MainState->get_Y_VelocityRel();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<get_X_VelocityRel>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost get_X_VelocityRel loop!" << e.what();
	}
	return 0;
}

float get_Z_VelocityRel()
{
	try
	{
		return MainState->get_Z_VelocityRel();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<get_Z_VelocityRel>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost get_Z_VelocityRel loop!" << e.what();
	}
	return 0;
}

float get_X_VelocityAbs()
{
	return MainState->get_X_VelocityAbs();
}

float get_Y_VelocityAbs()
{
	return MainState->get_Y_VelocityAbs();
}

float get_Z_VelocityAbs()
{
	return MainState->get_Z_VelocityAbs();
}

float getAltitude()
{
	//return MainState->getAltitude();
	try
	{
		return MainState->getAltitude();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getAltitude>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getHeadingDegrees() // Gives in Degrees
{
	//return MainState->getHeadingDegrees();
	try
	{
		return MainState->getHeadingDegrees();
	}

	catch (const std::future_error &e)
	{
		std::cout << "<getHeadingDegrees>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getHeading()
{
	//return MainState->getHeading();
	try
	{
		return MainState->getHeading();
	}

	catch (const std::future_error &e)
	{
		std::cout << "<getHeading>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

/***********************************************************************************************/
/******************************* A Little Higher Level Get APIs ********************************/
/***********************************************************************************************/

vector3D_t getVelocityAbs()
{
	return MainState->getVelocityAbs();
}

vector3D_t getVelocityRel()
{
	try
	{
		return MainState->getVelocityRel();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getVelocityRel>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost getVelocityRel loop!" << e.what();
	}
	return vector3D_t(0, 0, 0);
}

vector3D_t getVelocity() // CHANGE THIS
{
	try
	{
		return MainState->getVelocity();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getVelocity>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost getVelocity loop!" << e.what();
	}
	return vector3D_t(0, 0, 0);
}

GeoPoint_t getLocation() // CHANGE THIS
{
	try
	{
		return MainState->getLocalCoordinates();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getLocation>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return GeoPoint_t(0, 0, 0);
}

image_t getCameraView()
{
	return image_t();
}

data_imu_t getIMU()
{
	return data_imu_t(quaternion_t()); //(getAcc(), getGyro(), getMag());
}

DroneState_t getCompleteState()
{
	return DroneState_t(getIMU(), getVelocity(), getAltitude(), getHeading());
}

/******************************************************************************************/
/************************************ All the Set APIs ************************************/
/******************************************************************************************/

void _setPitch(int pitch)
{
	RC_APPARENT_DATA[PITCH] = (unsigned char)pitch;
#if defined(SYNCD_TRANSFER)
	sendCommand(pitch, 1);
#endif
}

void _setRoll(int roll)
{
	RC_APPARENT_DATA[ROLL] = (unsigned char)roll;
#if defined(SYNCD_TRANSFER)
	sendCommand(roll, 2);
#endif
}

void _setYaw(int yaw)
{
	RC_APPARENT_DATA[YAW] = (unsigned char)yaw;
#if defined(SYNCD_TRANSFER)
	sendCommand(yaw, 3);
#endif
}

void _setThrottle(int throttle)
{
	RC_APPARENT_DATA[THROTTLE] = (unsigned char)throttle;
#if defined(SYNCD_TRANSFER)
	sendCommand(throttle, 0);
#endif
}

void switchApparentRCstream()
{
	if (RC_APPARENT_DATA == (uint8_t *)RC_DATA)
		RC_APPARENT_DATA = (uint8_t *)RC_MASTER_DATA;
	else
		RC_APPARENT_DATA = (uint8_t *)RC_DATA;
}

void switchApparentRCstream(uint8_t *stream)
{
	RC_APPARENT_DATA = stream;
}

/*
	Actual Direct control functions =>
	(Includes Controller Overrides as well)
 */

void setRoll(float roll)
{
	RC_DATA[ROLL] = (unsigned char)roll + (RC_MASTER_DATA[ROLL] - 127);
}

void setPitch(float pitch)
{
	RC_DATA[PITCH] = (unsigned char)pitch + (RC_MASTER_DATA[PITCH] - 127);
}

void setThrottle(float throttle)
{
	RC_DATA[THROTTLE] = (unsigned char)throttle; // + (RC_MASTER_DATA[THROTTLE]);
}

void setYaw(float yaw)
{
	RC_DATA[YAW] = (unsigned char)yaw + (RC_MASTER_DATA[YAW] - 127);
}

void setAux1(int val)
{
	try
	{
		unsigned char t = (unsigned char)val;
		Main_Mutex.lock();
		// AUX1 to be used for PID Tuning, should set which
		RC_DATA[AUX1] = t;
#if defined(SYNCD_TRANSFER)
		sendCommand(val, 4);
#endif
		Main_Mutex.unlock();
		//IssueCommand();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setAux1>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		//Main_Mutex.unlock();
	}
}

void setAux2(int val)
{
	try
	{
		unsigned char t = (unsigned char)val;
		Main_Mutex.lock();
		RC_DATA[AUX2] = t;
#if defined(SYNCD_TRANSFER)
		sendCommand(val, 5);
#endif
		Main_Mutex.unlock();
		//IssueCommand();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setAux2>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		//Main_Mutex.unlock();
	}
}

void setAux3(int val)
{
	try
	{
		unsigned char t = (unsigned char)val;
		Main_Mutex.lock();
		RC_DATA[AUX3] = t;
#if defined(SYNCD_TRANSFER)
		sendCommand(val, 6);
#endif
		Main_Mutex.unlock();
		//IssueCommand();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setAux3>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		//Main_Mutex.unlock();
	}
}

void setAux4(int val)
{
	try
	{
		unsigned char t = (unsigned char)val;
		Main_Mutex.lock();
		RC_DATA[AUX4] = t;
#if defined(SYNCD_TRANSFER)
		sendCommand(val, 7);
#endif
		Main_Mutex.unlock();
		//IssueCommand();
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setAux4>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		//Main_Mutex.unlock();
	}
}

/***********************************************************************************************/
/******************************* A Little Higher Level Set APIs ********************************/
/***********************************************************************************************/

int setHeading(float heading)
{
	try
	{
		return setFeedbackYaw(heading);
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setHeading>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost setHeading loop!" << e.what();
	}
	return 0;
}

int setVelocity(const vector3D_t &val)
{
	try
	{
		setAltitude(val.z);
		return 0;
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setVelocity>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost setVelocity loop!" << e.what();
	}
	return 0;
}

int setAltitude(float altitude)
{
	try
	{
		if (IntentionOverride)
			return 1; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
		setFeedbackAltitude(altitude);
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setAltitude>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Z_Actuator loop!" << e.what();
	}

	return 0;
}

int setAutoYaw(float heading)
{
	if (IntentionOverride)
		return 1; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
	setFeedbackYaw(heading);

	return 0;
}

const float ROLL_RANGE = 190;

// Measurements-> Max Roll angle at 90 input = 28; Max Pitch = 50

int setAutoRoll(float angle)
{
	if (IntentionOverride)
		return 1; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
	//FeedbackControl::RollActuator.actuationControllerlock->lock();
	//FeedbackControl::RollActuator.setIntendedActuation(angle);
	//FeedbackControl::RollActuator.actuationControllerlock->unlock();

	// Convert 90, 0, -90 to 0, 127, 255
	//printf("<%d>", angle);
	float val = clamp(angle, -ROLL_RANGE, ROLL_RANGE, 255, 0);
	if (val < 0)
		val = 0;
	else if (val > 255)
		val = 255;
	setRoll(val);
	return 0;
}

const float PITCH_RANGE = 90;

int setAutoPitch(float angle)
{
	if (IntentionOverride)
		return 1; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
	//FeedbackControl::PitchActuator.actuationControllerlock->lock();
	//FeedbackControl::PitchActuator.setIntendedActuation(angle);
	//FeedbackControl::PitchActuator.actuationControllerlock->unlock();

	// Convert 90, 0, -90 to 0, 127, 255
	float val = clamp(angle, -PITCH_RANGE, PITCH_RANGE, 255, 0);
	if (val < 0)
		val = 0;
	else if (val > 255)
		val = 255;
	setPitch(val);
	return 0;
}

/***********************************************************************************************/
/*********************************** Other Higher Level APIs ***********************************/
/***********************************************************************************************/

void takeOff(float altitude)
{
	setAltitude(altitude);
	//setDestination(0, 0, 20);
}

/*
    High Level APIs 
*/

int setHeadingToPoint(GeoPoint_t destination)
{
	return setGazeOn(destination);
}

void holdPosition(float x, float y, float z)
{
	gotoLocation(x, y, z);
	// Unimplemented
}

GeoPoint_t lastDestination(0, 0, 0);

int AutoNavigate(const GeoPoint_t &destination, const GeoPoint_t &start = lastDestination, float max_velocity, bool override)
{
	try
	{
		printf("\n Got here!");
		AutoNavigation::Path_t *path;
		if (override)
		{
			path = AutoNavigation::makeLinearPath(getLocation(), destination, max_velocity);
		}
		else
		{
			path = AutoNavigation::makeLinearPath(start, destination, max_velocity);
		}
		getCurrentTrajectory()->addPath(path, override);
		printf("\nDone till here");
		fflush(stdout);
		lastDestination.set(destination);
		return path->id;
	}
	catch (const std::future_error &e)
	{
		std::cout << "<AutoNavigate>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Z_Actuator loop!" << e.what();
	}
	return 0;
}

int gotoLocation(float x, float y, float z, float max_velocity)
{
	printf("\n\n\nOVERRIDE Trying to move to position x=%f y=%f z=%f", x, y, z);
	return AutoNavigate(GeoPoint_t(x, y, z), getLocation(), max_velocity, true);
}

int addWaypoint(float x, float y, float z, float max_velocity)
{
	printf("\nTrying to move to position x=%f y=%f z=%f", x, y, z);
	return AutoNavigate(GeoPoint_t(x, y, z), lastDestination, max_velocity);
}

int returnToHome()
{
	return 0;
}

/*
    Some other Mechanisms
*/

// int unusedAPIhandler(std::vector<std::string> test)
// {
// 	std::cout << "\nOops! Seems someone sent me some wrong code!";
// 	return 1;
// }
int toggleFeedbackControllers(char type)
{
	try
	{
		printf("\nToggling Actuators...");
		fflush(stdout);
		if (IntentionOverride)
			IntentionOverride = false;
		else
			IntentionOverride = true;

		// int j = 0;
		// if (type == 'R')
		// 	j = 0;
		// else if (type == 'V')
		// 	j = 1;
		// else if (type == 'P')
		// 	j = 2;
		// else if (type == 'A')
		// {
		// 	/*if(IntentionOverride)
		//         IntentionOverride = false;
		//     else IntentionOverride = true;*/
		// 	toggleFeedbackControllers('R');
		// 	toggleFeedbackControllers('P');
		// 	toggleFeedbackControllers('V');
		// }
		// else
		// 	return 1;

		// if (!FeedbackControl::FeedbackControllerStatus[j])
		// {
		// 	FeedbackControl::FeedbackControllerStatus[j] = true;
		// 	printf("\tLocking...");
		// 	for (int i = 0; i < 3; i++)
		// 	{
		// 		FeedbackControl::FeedbackControllers[j][i]->actuationControllerlock->lock();
		// 	}
		// }
		// else
		// {
		// 	FeedbackControl::FeedbackControllerStatus[j] = false;
		// 	printf("\tUnlocking...");
		// 	for (int i = 0; i < 3; i++)
		// 	{
		// 		FeedbackControl::FeedbackControllers[j][i]->actuationControllerlock->unlock();
		// 	}
		// }
	}
	catch (const std::future_error &e)
	{
		std::cout << "<toggleFeedbackControllers>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some Error in toggleFeedbackControllers!" << e.what();
		fflush(stdout);
	}
	printf("\nDone...");
	fflush(stdout);
	return 0;
}

void MajorFailSafeMechanism()
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
		for (int i = a; i > 100; i--)
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
		for (int i = 100; i > 30; i--) // 2 Seconds
		{
			failsafe.lock();
			if (FaultManaged)
			{
				failsafe.unlock();
				return;
			}
			std::cout << "\nWaiting for connection while Landing... " << i;
			setThrottle(i);
			failsafe.unlock();
			std::this_thread::sleep_for(std::chrono::milliseconds(FAILSAFE_LANDING_RATE));
		}
		// Disarm and send FailSafe!
		FailSafeTrigger = true;
		setAux1(51); // Trigger Failsafe
		std::cout << "\nFailSafe Locked!";
		Main_Mutex.lock(); // Grab the lock and don't release until the fault is fixed
	}
	catch (const std::future_error &e)
	{
		std::cout << "<FailSafeMechanism>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some More Error 2!" << e.what();
	}
}

void FailSafePositionHold()
{
	// Change the code to use actual position hold!
	setYaw(127);
	setRoll(127);
	setPitch(127);
	setYaw(127);

	setThrottle(100);
}

void ConnectionFailSafeMechanism()
{
	try
	{
		// Decrease the throttle at constant rate to land
		std::cout << "\nInitiating FailSafe...";
		// Disarm and send FailSafe!
		FailSafePositionHold();
		while (!FaultManaged)
		{
			std::cout << "\nWaiting for connection...";
			std::this_thread::sleep_for(std::chrono::milliseconds(FAILSAFE_LANDING_RATE));
		}
		std::cout << "\nFailsafe Managed!";
		// std::cout << "\nFailSafe Locked!";
		// Main_Mutex.lock(); // Grab the lock and don't release until the fault is fixed
	}
	catch (const std::future_error &e)
	{
		std::cout << "<ConnectionFailSafeMechanism>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some More Error 2!" << e.what();
	}
}

void ConnectionFaultResume()
{
	//#if defined(MSP_Serial_PROTOCOL)
	try
	{
		// failsafe.lock();
		FaultManaged = true;
		// failsafe.unlock();
		FailSafeThread->join();
		FailSafeTrigger = false;
		// if (FailSafeTrigger)
		// {
		// 	// Main_Mutex.unlock();
		// 	FailSafeTrigger = false;
		// }
		std::cout << "Fault Resumed and Managed!\n";
	}
	catch (const std::future_error &e)
	{
		std::cout << "<ResumeHandler>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some More Error 3!" << e.what();
	}
	//#endif
}

void ConnectionFaultHandler()
{
	//#if defined(MSP_Serial_PROTOCOL)
	try
	{
		if(FailSafeTrigger) return;
		std::cout << "Fault Occured!\nTriggering FailSafe!!!";
		FaultManaged = false;
		FailSafeTrigger = true;
		FailSafeThread = new std::thread(ConnectionFailSafeMechanism);
	}
	catch (const std::future_error &e)
	{
		std::cout << "<FaultHandler>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some More Error 1!" << e.what();
	}
	//#endif
}

/*
    The Main FlightController Interfacing Loop
*/

#if defined(UNIFIED_UPDATER)

void FCboard_InterfaceLoop()
{
	// This loop runs at a clock of once per every Milisecond.
	uint counter = 0, c2 = 0;
	std::chrono::high_resolution_clock::time_point start_time;
	std::thread sensors(Sensors_Updater);
	int loop_time;
	try
	{
		while (1)
		{
			start_time = std::chrono::high_resolution_clock::now();
			// ++c2;
			// if(c2 == 100)
			// {
			// 	printf("\nTICK");
			// 	fflush(stdout);
			// 	c2 = 0;
			// }

			Main_Mutex.lock();
			// if (counter%10 == 0)
			Channel_Updater(); // Sends computed RC data to FC
			if (counter >= CLI_UPDATE_RATE)
			{
				Channel_ViewRefresh(); // User Interface
				counter = 0;
			}
			Main_Mutex.unlock();
			++counter;
			const auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();

			loop_time = 1000 - total_time;
			// std::cout<<"\nLoopTime : "<<total_time<<" us";
			if (loop_time < 0)
			{
				loop_time = 10;
				// std::cout<<"\nLoopTime : "<<total_time<<" us";
			}
			std::this_thread::sleep_for(std::chrono::microseconds(loop_time));
			// std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	catch (std::exception &e)
	{
		std::cout << "\nError in FCboard_InterfaceLoop => " << e.what();
		exit(0);
	}
	sensors.join();
}

#endif

/*
    The Main Initializer Function
*/

int ControllerInterface_init(int argc, const char *argv[])
{
	try
	{
		printf("\n Initializing Flight Controller Interface...");
		//int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
		Raw_Init(argc, argv);

		// IntentionOverride = false;
		disableAutoNav(); // By default, Auto navigation is disabled
		switchApparentRCstream((uint8_t *)RC_DATA);

#if defined(MODE_AIRSIM)

		// #include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
		// #include "rpc/server.h"
		//auto client = (msr::airlib::MultirotorRpcLibClient *)MainFC->getDesc();
		MainState = new AirSim_StateEstimator_t();

#elif defined(MODE_REALDRONE)
		// Sensor_Fusion_init(argc, (char **)argv);
		MainState = new Real_StateEstimator_t();
#endif


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

		setAltitude(0);

#if defined(CLI_MONITOR)

#if !defined(UNIFIED_UPDATER)
		chnl_refresh = new std::thread(Channel_ViewRefresh);
#endif
		for (int i = 0; i < 256; i++)
			KeyMap[i] = event_key_other;
		KeyMap['q'] = event_key_q; // Show RC Values
		KeyMap['w'] = event_key_w; // show PID Values
		KeyMap['e'] = event_key_e; // show MainState Data
		KeyMap['r'] = event_key_r; // show Wifi Strength
		KeyMap['A'] = event_key_A; // show Armed
		KeyMap['P'] = event_key_P; // show Position
		KeyMap['V'] = event_key_V; // show Velocity
		KeyMap['h'] = event_key_h; // set PIDs
		KeyMap['s'] = event_key_s; // show Status
		keyboard_handler = new std::thread(Keyboard_handler, 2);
#endif
#if defined(UNIFIED_UPDATER)
		chnl_update = new std::thread(FCboard_InterfaceLoop);
#elif defined(UPDATER_THREAD)
		chnl_update = new std::thread(Channel_Updater);
#endif

		AutoNavigation::initialize_AutoNavigation();
		return 0;
	}
	catch (const std::future_error &e)
	{
		std::cout << "<ControllerInterface_init>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		//Main_Mutex.unlock();
	}
	return 1;
}

} // namespace ControllerInterface
