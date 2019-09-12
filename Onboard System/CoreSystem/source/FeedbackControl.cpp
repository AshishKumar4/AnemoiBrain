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
#include <atomic>

#include "Sensors/Sensors.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "Sensors/Location.hpp"

#include "ControllerInterface.hpp"
#include "CommonControl.hpp"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------General Purpose tools---------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

namespace ControllerInterface
{

/*
    GPS/Barometer/Compass assisted
    All SI Units, Meters, Degrees
    Forward pitch direction in Y Axis, Left Right is X axis, up is Z
*/

float getPathDeviation();
float getPathLength();
void setForwardComponent(float vel);
void setSidewaysComponent(float val);

float E_VALUE = 0;

namespace FeedbackControl
{

vector3D_t *controlledPosition;
vector3D_t *controlledOrientation;

std::thread *YawControllerThread;
std::thread *RollControllerThread;
std::thread *PitchControllerThread;
std::thread *AltitudeControllerThread;

std::thread *X_Vheadfree_ControllerThread;
std::thread *Y_Vheadfree_ControllerThread;
std::thread *Z_Vheadfree_ControllerThread;

std::thread *X_Vrel_ControllerThread;
std::thread *Y_Vrel_ControllerThread;
std::thread *Z_Vrel_ControllerThread;

std::thread *X_Lateral_ControllerThread;
std::thread *Y_Lateral_ControllerThread;
std::thread *Z_Lateral_ControllerThread;

int DefaultEscapeFunction(float error, float dval)
{
	return 0;
}

class FeedbackController_t // An Abstract class
{
	std::atomic<float> IntendedActuation;

protected:
	float errorScale;
	float actuateVal;
	float currentactuation;
	float oldError;
	float oldAbsError;
	double deltaTime;
	float errorAccumulator;

public:
	std::thread *actuatorThread;
	float CONTROLLER_P;
	float CONTROLLER_I;
	float CONTROLLER_D;
	float CONTROLLER_E;
	float CONTROLLER_E_RANGE;
	float MAX_I_BOUNDARY;

	float ACTUATION_HALT_VALUE;
	float ACTUATION_MAX_VALUE;
	float ACTUATION_MIN_VALUE;

	int RC_Channel;

	std::mutex *intentionLock;
	std::mutex *actuationControllerlock;

	std::function<void(float)> setActuation;
	std::function<float(void)> getCurrentStateValues;
	std::function<float(float)> ErrorProcessor;
	std::function<int(float, float)> EscapeFunction;

	FeedbackController_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200)
	{
		setActuation = actuatorSet;
		getCurrentStateValues = actuatorGet;
		IntendedActuation = 0;
		CONTROLLER_P = CONSTANT_P;
		CONTROLLER_I = CONSTANT_I;
		CONTROLLER_D = CONSTANT_D;
		CONTROLLER_E = 0;

		MAX_I_BOUNDARY = 150;
		intentionLock = new std::mutex();
		actuationControllerlock = new std::mutex();

		RC_Channel = rcChannel;
		EscapeFunction = DefaultEscapeFunction;
	}

	int setIntendedActuation(float intention)
	{
		try
		{
//intentionLock->lock();
#if defined(ACTUATION_INTENTION_RELATIVE)
			// The Intended actuation should be relative to the current state
			IntendedActuation = float(this->getCurrentStateValues() + intention);
#else
			IntendedActuation = intention;
#endif
			//intentionLock->unlock();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<setIntendedActuation>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
		}
		catch (const std::exception &e)
		{
			std::cout << e.what() << '\n';
		}

		return 0;
	}

	float getIntendedActuation()
	{
		try
		{
			float intention;
#if defined(ACTUATION_INTENTION_RELATIVE)
			//intentionLock->lock();
			intention = IntendedActuation;
#else
			intention = IntendedActuation;
#endif
			//if(IntentionOverride)
			//    intention = RC_MASTER_DATA[this->RC_Channel];
			//intentionLock->unlock();
			return intention;
		}
		catch (const std::future_error &e)
		{
			std::cout << "<getIntendedActuation>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
		}
		catch (const std::exception &e)
		{
			std::cout << e.what() << '\n';
		}
		return 0;
	}

	virtual void deployFeedbackControllers() = 0;

	float FeedbackController()
	{
		try
		{
			if (IntentionOverride)
			{
				return -1; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
			}
			//this->actuationControllerlock->lock();*/
			float h = this->getCurrentStateValues();
			float newError = h - this->getIntendedActuation();

			// Any processing needed?
			newError = ErrorProcessor(newError);
			//printf("\n{>>%f, \t%f<<},", h, newError);

			if (this->EscapeFunction(newError, oldError))
			{
				this->actuationControllerlock->unlock();
				return -1;
			}

			// Our Equation
			float P_term = newError * this->CONTROLLER_P; //clamp(newError * errorScale, -180, 180, -127, 127);
			this->errorAccumulator += double(newError) * deltaTime * 0.001;
			if (this->errorAccumulator)
			{
				this->errorAccumulator = tanh(this->errorAccumulator / this->MAX_I_BOUNDARY) * this->MAX_I_BOUNDARY;
			}
			float I_term = float(this->errorAccumulator * this->CONTROLLER_I);
			double derivative = (double(newError - oldError) / (deltaTime));
			float D_term = derivative * this->CONTROLLER_D;
			//float E_term = (derivative / double(newError)) * this->CONTROLLER_E;
			float E_term = derivative * (1 - tanh(abs(double(newError) * this->CONTROLLER_E_RANGE))) * this->CONTROLLER_E;
			//float E_term = (derivative / (double(newError)*double(newError))) * this->CONTROLLER_E;

			//if (this->CONTROLLER_E)
			{
				//printf(" <%f> ", E_term);
			}
			//printf("\n{%f}", D_term);
			//if(newError != 0)
			//    D_term = (D_term) ;

			actuateVal = P_term + I_term + D_term + E_term + RC_MASTER_DATA[this->RC_Channel]; //currentYaw; //(newError/error); // - (newError / oldError)*(absNewError/newError)*2.0 // 2*(oldAbsError - 1*absNewError)

			//printf("=> P:%f, I:%f, D:%f, => %f", P_term, I_term, D_term, actuateVal);
			if (actuateVal > this->ACTUATION_MAX_VALUE)
				actuateVal = this->ACTUATION_MAX_VALUE;
			else if (actuateVal < this->ACTUATION_MIN_VALUE)
				actuateVal = this->ACTUATION_MIN_VALUE;
			//printf(", %f", actuateVal);
			this->setActuation(actuateVal);
			oldError = newError;
			//oldAbsError = absNewError;
			//this->actuationControllerlock->unlock();
			std::this_thread::sleep_for(std::chrono::microseconds(int(deltaTime * 1000)));

			return D_term; // Useful to determine if the drone is stationary
		}
		catch (const std::future_error &e)
		{
			std::cout << "<FeedbackController>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			this->actuationControllerlock->unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error Occured! inside " << e.what();
			this->actuationControllerlock->unlock();
		}
		return -1;
	}
};

volatile FeedbackController_t *FeedbackControllers[3][3];
volatile bool FeedbackControllerStatus[3];

/* ------------------------------------------------------------------------------------------------------------------------ */
/*-------------------------------------------------- RotationalControllers -------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

volatile inline float DegreeRoundclamp(float val)
{
	float decimal = val - float(int(val));
	val = float(int(val) % 360) + decimal;
	if (val > 180 || val < -180)
	{
		val -= 360;
	}
	return val;
}

class RotationController_t : public FeedbackController_t
{
public:
	RotationController_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
	{
		errorScale = CONTROLLER_P;
		actuateVal = 127;
		currentactuation = 127;
		oldError = 1;
		oldAbsError = 1;
		deltaTime = 1;

		ACTUATION_HALT_VALUE = 127;
		ACTUATION_MAX_VALUE = 255;
		ACTUATION_MIN_VALUE = 0;

		this->MAX_I_BOUNDARY = 1000;

		this->CONTROLLER_P = 0.8;
		this->CONTROLLER_I = 0;
		this->CONTROLLER_D = 10;
		this->CONTROLLER_E = 0;

		ErrorProcessor = DegreeRoundclamp;
	}

	void deployFeedbackControllers()
	{
	}

	/*void AutoRotationalController()
    {
        FeedbackController();
    }*/
};

RotationController_t YawActuator(setYaw, getHeadingDegrees, YAW);
RotationController_t RollActuator(setRoll, getRollDegrees, ROLL);
RotationController_t PitchActuator(setPitch, getPitchDegrees, PITCH);

void YawController()
{
	while (true)
	{
		try
		{
			YawActuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<YawController>Outermost<->Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			YawActuator.actuationControllerlock->unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost Yaw loop!" << e.what();
		}
	}
}

void RollController()
{
	while (true)
	{
		try
		{
			RollActuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<RollController>Outermost<->Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			RollActuator.actuationControllerlock->unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost Roll loop!" << e.what();
		}
	}
}

void PitchController()
{
	while (true)
	{
		try
		{
			PitchActuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<PitchController>Outermost<->Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			PitchActuator.actuationControllerlock->unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost Pitch loop!" << e.what();
		}
	}
}

/*
    APIs that can be built over these functions
*/

int init_RotationalControllers()
{
	YawActuator.CONTROLLER_P = 0.7;   //ku/5;//2.3;//6.7;//11;
	YawActuator.CONTROLLER_I = 0.004; //(float(2/5)*ku)/tu;//0.0001;
	YawActuator.CONTROLLER_D = 210;   //(ku*tu)/15.0;//1000;//190;
	RollActuator.CONTROLLER_P = 1;
	YawControllerThread = new std::thread(YawController);
	//RollControllerThread = new std::thread(RollController);
	//PitchControllerThread = new std::thread(PitchController);
	return 0;
}

int join_RotationalControllers()
{
	YawControllerThread->join();
	// RollControllerThread->join();
	// PitchControllerThread->join();
	return 0;
}

int destroy_RotationalControllers()
{
	return 0;
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*--------------------------------------------- LateralControllers (Positional) --------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

float InverseErrorScaler(float val) // Use a function
{
	return -val; //- tanh(val/max_scale) * max_scale;
}

float IdentityErrorScaler(float val) // Use a function
{
	//printf("Yerr: %f\t", val);
	return val; //- tanh(val/max_scale) * max_scale;
}

int PositionHoldEscapeFunction(float error, float dval)
{
	//if(abs(error) <= 3)// && abs(error - dval) <= 10)
	//    return 1;
	return 0;
}

class Planner_X_Controller_t : public FeedbackController_t
{
public:
	Planner_X_Controller_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
	{
		actuateVal = 0;			   //127;
		currentactuation = 0;	  // 127;
		ACTUATION_HALT_VALUE = 0;  //127;
		ACTUATION_MAX_VALUE = 90;  //0;
		ACTUATION_MIN_VALUE = -90; //255;

		oldError = 1;
		oldAbsError = 1;
		deltaTime = 1;

		this->MAX_I_BOUNDARY = 100;
		this->CONTROLLER_P = 15;
		this->CONTROLLER_I = 1;
		this->CONTROLLER_D = 10;
		this->CONTROLLER_E = 0;		  //20000;
		this->CONTROLLER_E_RANGE = 0; //.08;

		ErrorProcessor = IdentityErrorScaler;
	}

	void deployFeedbackControllers()
	{
		//actuatorThread = new std::thread(this->AutoLateralalActuator);
	}

	void AutoLateralalActuator()
	{
		FeedbackController();
	}
};

class Planner_Y_Controller_t : public FeedbackController_t
{
public:
	Planner_Y_Controller_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
	{
		actuateVal = 0;			   //127;
		currentactuation = 0;	  // 127;
		ACTUATION_HALT_VALUE = 0;  //127;
		ACTUATION_MAX_VALUE = 90;  //0;
		ACTUATION_MIN_VALUE = -90; //255;

		oldError = 1;
		oldAbsError = 1;
		deltaTime = 1;

		this->MAX_I_BOUNDARY = 1000;
		this->CONTROLLER_P = 3;
		this->CONTROLLER_I = 0.1;
		this->CONTROLLER_D = 1000;
		this->CONTROLLER_E_RANGE = 0.20;
		this->CONTROLLER_E = 10000;

		ErrorProcessor = IdentityErrorScaler;
	}

	void deployFeedbackControllers()
	{
		//actuatorThread = new std::thread(this->AutoLateralalActuator);
	}

	void AutoLateralalActuator()
	{
		FeedbackController();
	}
};

class LateralController_t : public FeedbackController_t
{
public:
	LateralController_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
	{
		actuateVal = 0;			   //127;
		currentactuation = 0;	  // 127;
		ACTUATION_HALT_VALUE = 0;  //127;
		ACTUATION_MAX_VALUE = 90;  //255;
		ACTUATION_MIN_VALUE = -90; //0;

		oldError = 1;
		oldAbsError = 1;
		deltaTime = 10;

		this->MAX_I_BOUNDARY = 1000;
		this->CONTROLLER_P = 3;
		this->CONTROLLER_I = 0.1;
		this->CONTROLLER_D = 1000;
		this->CONTROLLER_E_RANGE = 0.20;
		E_VALUE = 10000;
		//this->CONTROLLER_E = 100000;

		ErrorProcessor = IdentityErrorScaler;
		//EscapeFunction = PositionHoldEscapeFunction;
	}

	void deployFeedbackControllers()
	{
		//actuatorThread = new std::thread(this->AutoLateralalActuator);
	}

	void AutoLateralalActuator()
	{
		FeedbackController();
	}
};

class AltitudeController_t : public FeedbackController_t
{
public:
	AltitudeController_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
	{
		actuateVal = 127;
		currentactuation = 0;
		ACTUATION_HALT_VALUE = 127;
		ACTUATION_MAX_VALUE = 255;
		ACTUATION_MIN_VALUE = 0;

		oldError = 0;
		oldAbsError = 0;
		deltaTime = 1;

		this->MAX_I_BOUNDARY = 1000;
		this->CONTROLLER_P = 40;
		this->CONTROLLER_I = 0.01;
		this->CONTROLLER_D = 10000;

		ErrorProcessor = InverseErrorScaler;
	}

	void deployFeedbackControllers()
	{
		//actuatorThread = new std::thread(this->AutoLateralalActuator);
	}

	void AutoLateralalActuator()
	{
		FeedbackController();
	}
};

//Planner_X_Controller_t Xrel_Actuator(setAutoPitch, getDesiredVelocity, RC_X_MOTION); // instead of getPathLength, use clamped value of it

// The X actuator is indeed a Velocity Controller!
// Planner_X_Controller_t Xrel_Actuator(setAutoPitch, getForwardVelocity, RC_X_MOTION);
// Planner_Y_Controller_t Yrel_Actuator(setAutoRoll, getPathDeviation, RC_Y_MOTION);
Planner_X_Controller_t Xrel_Actuator(setForwardComponent, getForwardVelocity, RC_X_MOTION);
Planner_Y_Controller_t Yrel_Actuator(setSidewaysComponent, getPathDeviation, RC_Y_MOTION);
AltitudeController_t Z_Actuator(setThrottle, getAltitude, THROTTLE);

LateralController_t Distance_Actuator(HeadlessHover, getDesiredVelocity, RC_X_MOTION);

void Z_Lateral_Controllers()
{
	while (true)
	{
		try
		{
			//printf(">_<");
			Z_Actuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<Z_Lateral_Controllers>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost Z_Actuator loop!" << e.what();
		}
	}
}

int init_LateralControllers()
{
	//Y_Actuator.ErrorProcessor = Planner_Y_ControllerErrorScaler;
	Z_Lateral_ControllerThread = new std::thread(Z_Lateral_Controllers);

	return 0;
}

int join_LateralControllers()
{
	Z_Lateral_ControllerThread->join();

	return 0;
}

int destroy_LateralControllers()
{
	//YawControllerThread->join();
	return 0;
}

} // namespace FeedbackControl

/* Our FeedbackControl Namespace ends here, Autonomous APIs Below */

void X_Position_Hold()
{
	while (true)
	{
		try
		{
			//FeedbackControl::X_Actuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<X_Lateral_Controllers>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost X_Actuator loop!" << e.what();
		}
	}
}

void Y_Position_Hold()
{
	while (true)
	{
		try
		{
			//FeedbackControl::Y_Actuator.FeedbackController();
		}
		catch (const std::future_error &e)
		{
			std::cout << "<Y_Lateral_Controllers>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
		}
	}
}

namespace
{

float pathDestinationX;
float pathDestinationY;
float pathDestinationZ;

float pathStartX;
float pathStartY;
float pathStartZ;

float totalDistance;
float deltaX;
float deltaY;
float pathConstant;

float olderDistance;
float initialHeading;

std::atomic<float> desiredHeading;
std::atomic<float> currentDistance;
std::atomic<float> currentDeviation;

std::atomic<float> TEMP_FORWARD_COMPONENT;
std::atomic<float> TEMP_DRIFT_COMPONENT;

std::mutex PathFollowingLoopLock;

std::atomic<bool> exitPathFollowingLoop;
std::atomic<int> exitCourseDistanceController;
std::atomic<int> exitCourseDeviationController;

std::atomic<bool> shouldHoldPosition = false;
std::atomic<bool> hoverStableFlag = false;

} // namespace

std::thread *moveThread, *gazeThread, *holdPositionThread;

/*
    Convention ==>
        F
    L   -   R
        B

    Pitch -->
    -90
     |
     |
     =
     |
     |
     +90

     Roll -->
     +90 ---|--- -90
*/

float MAX_DESIRED_VELOCITY = 15;
float CLAMP_FACTOR = 30;
// For primary Path Following ==>
#define TRANSITION_DISTANCE 3
#define MAX_DEVIATION_ALLOWED 5

// These parameters govern action if hover gets unstable ==>
#define SECONDARY_TRANSITION_DISTANCE 3
#define MAX_HOVER_DEVIATION_ALLOWED 5

float getDesiredVelocity()
{
	float val = getPathLength();
	return tanh(val / CLAMP_FACTOR) * MAX_DESIRED_VELOCITY;
}

float getForwardVelocity()
{
	vector3D_t v = getVelocityAbs();			  //get_Y_VelocityRel();
	float theta = -degreesToRads(desiredHeading); //getHeading();;
	float _v = v.y * cos(theta) + v.x * sin(theta);
	printf("\t<<%f, [%f, %f]; %f>>", _v, theta, getHeading(), get_Y_VelocityRel());
	return (_v);
}

void setForwardComponent(float vel)
{
	TEMP_FORWARD_COMPONENT = vel;
}

void setSidewaysComponent(float val)
{
	TEMP_DRIFT_COMPONENT = val;
}

float getPathLength()
{
	try
	{
		return currentDistance;
	}

	catch (const std::future_error &e)
	{
		std::cout << "<getPathLength>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

float getPathDeviation()
{
	try
	{ // This should give me the deviation
		return currentDeviation;
	}
	catch (const std::future_error &e)
	{
		std::cout << "<getPathDeviation>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

inline float euclideanDistance2d(float x2, float y2, float x1, float y1)
{
	return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

inline float calcPathDeviation(GeoPoint_t currentLocation)
{
	if (totalDistance)
		return ((deltaY * currentLocation.x) - (deltaX * currentLocation.y) + (pathConstant)) / totalDistance;
	return 0;
}

int set2DLinearPath(float startX, float startY, float destX, float destY)
{
	pathDestinationX = destX;
	pathDestinationY = destY;

	pathStartX = startX;
	pathStartY = startY;

	// Compute initial heading for future computation

	float dY = pathDestinationY - pathStartY;
	float dX = pathDestinationX - pathStartX;
	float lenXY = sqrt(pow(dX, 2) + pow(dY, 2));
	totalDistance = lenXY;
	deltaX = dX;
	deltaY = dY;
	currentDistance = lenXY;
	pathConstant = pathDestinationX * pathStartY - pathDestinationY * pathStartX;
	return 0;
}

int setLinearPath(GeoPoint_t start, GeoPoint_t destination, float cruise_velocity, float final_velocity, float clamp_factor)
{
	try
	{
		pathDestinationZ = destination.z;
		pathStartZ = start.z;
		MAX_DESIRED_VELOCITY = cruise_velocity;
		CLAMP_FACTOR = clamp_factor;

		set2DLinearPath(start.x, start.y, destination.x, destination.y);
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

void distanceController()
{
	while (true)
	{
		try
		{
			if (exitCourseDistanceController)
				return;
			FeedbackControl::Xrel_Actuator.FeedbackController();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost X_Actuator loop!" << e.what();
		}
	}
}

void deviationController()
{
	while (true)
	{
		try
		{
			if (exitCourseDeviationController)
				return;
			FeedbackControl::Yrel_Actuator.FeedbackController();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Outermost X_Actuator loop!" << e.what();
		}
	}
}

void HeadlessHover(float val)
{
	float dTheta = degreesToRads(desiredHeading) - (-getHeading());
	float pitch = -cosf(dTheta) * val;
	float roll = sinf(dTheta) * val;
	setAutoPitch(pitch);
	setAutoRoll(roll);
}

inline void HeadlessMoveTowardsTarget()
{
	float dTheta = degreesToRads(desiredHeading) - (-getHeading());
	float pitch = (cosf(dTheta) * TEMP_FORWARD_COMPONENT) + (sinf(dTheta) * TEMP_DRIFT_COMPONENT);
	float roll = (-sinf(dTheta) * TEMP_FORWARD_COMPONENT) + (cosf(dTheta) * TEMP_DRIFT_COMPONENT);
	setAutoRoll(roll);
	setAutoPitch(pitch);
	//printf("{{%f, %f}}", TEMP_DRIFT_COMPONENT, roll);
}

inline void updateTargetDistanceDirection(GeoPoint_t currentLocation = getLocation())
{
	// Firstly move to the direction to face the destination
	float dY = pathDestinationY - currentLocation.y;
	float dX = pathDestinationX - currentLocation.x;
	float lenXY = sqrt(pow(dX, 2) + pow(dY, 2));

	olderDistance = currentDistance;
	float heading = copysign(acos(dY / lenXY), -dX);
	float headingDegrees = (heading * 180) / PI;

	if (isnan(headingDegrees))
		return;

	desiredHeading = headingDegrees;
	currentDistance = lenXY;
}

inline void updateTargetStatus(GeoPoint_t currentLocation = getLocation())
{
	updateTargetDistanceDirection(currentLocation);
	// Using Distance of point from line for deviation -->
	float dev = calcPathDeviation(currentLocation);

	float lenXY = currentDistance;
	float desiredVel = getDesiredVelocity();
	printf("\n{%f},\t{%f}\t[%f]", lenXY, dev, desiredVel);
	currentDeviation = dev;
	FeedbackControl::YawActuator.setIntendedActuation(desiredHeading);
	FeedbackControl::Xrel_Actuator.setIntendedActuation(desiredVel);
	HeadlessMoveTowardsTarget();
}

inline void PathFollowingLoop(float transition_distance = TRANSITION_DISTANCE);

void holdPositionLoop()
{
	while (1)
	{
		if (shouldHoldPosition)
		{
			updateTargetDistanceDirection();
			float motion = FeedbackControl::Distance_Actuator.FeedbackController();
			float len = currentDistance;
			printf("\n{%f}", len);

			if (len >= MAX_HOVER_DEVIATION_ALLOWED)
			{
				PathFollowingLoop(SECONDARY_TRANSITION_DISTANCE);
			}
			if (!hoverStableFlag && len < 2.5 && motion == 0)
			{
				hoverStableFlag = true;
			}
		}
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}
}

inline void PathFollowingLoop(float transition_distance)
{
	// exitPathFollowingLoop = false;
	PathFollowingLoopLock.lock();

	exitCourseDistanceController = 0;
	exitCourseDeviationController = 0;

	std::thread *distanceControllerThread = new std::thread(distanceController);
	std::thread *deviationControllerThread = new std::thread(deviationController);
	while (1)
	{
		updateTargetStatus();
		// If deviation is too much, stop and fix it first!
		if (abs(currentDeviation) >= MAX_DEVIATION_ALLOWED)
		{
			printf("\n\nChanging Start Position");
			exitCourseDeviationController = 1;
			deviationControllerThread->join();
			exitCourseDeviationController = 0;
			GeoPoint_t curr = getLocation();
			set2DLinearPath(curr.x, curr.y, pathDestinationX, pathDestinationY);
			deviationControllerThread = new std::thread(deviationController);
		}

		// /printf("<<<<%d>>>>", (int)exitPathFollowingLoop);

		if (currentDistance <= transition_distance || exitPathFollowingLoop)
		{
			exitCourseDistanceController = 1;
			exitCourseDeviationController = 1;
			distanceControllerThread->join();
			deviationControllerThread->join();
			std::cout << "\nWe are here";
			fflush(stdout);
			break;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}

	PathFollowingLoopLock.unlock();
}

void executeLinearPathFollowing()
{
	shouldHoldPosition = false;
	updateTargetStatus();

	terminateActiveFollowing();
	exitPathFollowingLoop = false;

	std::this_thread::sleep_for(std::chrono::milliseconds(1500));

	PathFollowingLoop(); // Wait for it to finish

	if (!exitPathFollowingLoop)
	{
		// If path following wasn't terminated intentionally!
		printf("\nEngaging Position Hold System");
		FeedbackControl::Distance_Actuator.CONTROLLER_E = E_VALUE;
		shouldHoldPosition = true; // Give control over to Hold Position loop
		hoverStableFlag = false;
		while (!hoverStableFlag)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	else
	{
		printf("\nPathFollowing Intentionally Terminated!");
	}
}

int setGazeOn(GeoPoint_t destination)
{
	try
	{
		pathDestinationX = destination.x;
		pathDestinationY = destination.y;
		pathDestinationZ = destination.z;
	}
	catch (const std::future_error &e)
	{
		std::cout << "<setGazeOn>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
	}
	return 0;
}

int setFeedbackAltitude(float altitude)
{
	FeedbackControl::Z_Actuator.actuationControllerlock->lock();
	FeedbackControl::Z_Actuator.setIntendedActuation(altitude);
	printf("\n>>%f", altitude);
	FeedbackControl::Z_Actuator.actuationControllerlock->unlock();
	return 0;
}

int setFeedbackYaw(float heading)
{
	FeedbackControl::YawActuator.setIntendedActuation(heading);
	return 0;
}

int moveSavedPath()
{
	// moveThread = new std::thread(executeLinearPathFollowing);
	// moveThread->join();
	executeLinearPathFollowing();
	return 0;
}

int terminateActiveFollowing()
{
	exitPathFollowingLoop = true;
	return 0;
}

int launch_ActuationControllers()
{
	try
	{
#if defined(AUTONOMOUS_ACTUATION_CONTROLLERS)
		FeedbackControl::init_RotationalControllers();
		//FeedbackControl::init_DroneVelocityControllers();
		FeedbackControl::init_LateralControllers();

		holdPositionThread = new std::thread(holdPositionLoop);
		// Wait for them to join
		FeedbackControl::join_RotationalControllers();
		FeedbackControl::join_LateralControllers();
		holdPositionThread->join();
		//FeedbackControl::join_DroneVelocityControllers();
#endif
	}
	catch (const std::future_error &e)
	{
		std::cout << "<launch_ActuationControllers>Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (const std::exception &e)
	{
		std::cout << e.what() << '\n';
	}

	return 0;
}

int destroy_ActuationControllers()
{
	FeedbackControl::destroy_RotationalControllers();
	FeedbackControl::destroy_LateralControllers();
	return 0;
}

} // namespace ControllerInterface