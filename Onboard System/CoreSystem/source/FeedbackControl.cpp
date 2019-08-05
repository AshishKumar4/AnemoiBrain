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
std::atomic<bool> IntentionOverride;

float getPathLength();
float getPathDeviation();

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

float Controlled_IntendedYawHeading = 0;
float Controlled_IntendedPitchHeading = 0;
float Controlled_IntendedRollHeading = 0;
float Controlled_IntendedAltitude = 0;

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
    float deltaTime;
    float errorAccumulator;

public:
    std::thread *actuatorThread;
    float CONTROLLER_P;
    float CONTROLLER_I;
    float CONTROLLER_D;
    float MAX_I_BOUNDARY;

    float ACTUATION_HALT_VALUE;
    float ACTUATION_MAX_VALUE;
    float ACTUATION_MIN_VALUE;

    int RC_Channel;

    std::mutex *intentionLock;
    std::mutex *actuationControllerlock;

    std::function<void(int)> setActuation;
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

    void FeedbackController()
    {
        try
        {
            if (IntentionOverride)
            {
                return; // If the Feedback Controllers are overriden by the user manually, Do not attempt anything
            }
            this->actuationControllerlock->lock();
            float h = this->getCurrentStateValues();
            float newError = h - this->getIntendedActuation();
            if (h == newError && !h)
            {
                this->actuationControllerlock->unlock();
                std::this_thread::sleep_for(std::chrono::microseconds(int(deltaTime * 1000)));

                return;
            }
            // Any processing needed?
            //printf("\n{>>%f, \t%f<<},", h, newError);
            newError = ErrorProcessor(newError);

            if(this->EscapeFunction(newError, oldError))
            {      
                this->actuationControllerlock->unlock();
                return;
            }

            /*float absNewError = std::abs(newError);
            if (absNewError <= 2)
            {
                //if (this->ACTUATION_HALT_VALUE >= 0)
                //    this->setActuation(this->ACTUATION_HALT_VALUE); // Make it not move anymore
                //printf("\nDone...%f %f", newError, h);            
                this->actuationControllerlock->unlock();
                return;
            }*/
            // Our Equation
            float P_term = newError * this->CONTROLLER_P; //clamp(newError * errorScale, -180, 180, -127, 127);
            float I_term = (this->errorAccumulator + (newError * (this->CONTROLLER_I)));
            this->errorAccumulator = I_term;
            if (this->errorAccumulator)
            {
                this->errorAccumulator = tanh(this->errorAccumulator / this->MAX_I_BOUNDARY) * this->MAX_I_BOUNDARY;
            }

            float D_term = ((newError - oldError) / (deltaTime)) * this->CONTROLLER_D;

            actuateVal = P_term + I_term + D_term + RC_MASTER_DATA[this->RC_Channel]; //currentYaw; //(newError/error); // - (newError / oldError)*(absNewError/newError)*2.0 // 2*(oldAbsError - 1*absNewError)

            //printf("=> P:%f, I:%f, D:%f, => %f", P_term, I_term, D_term, actuateVal);
            if (actuateVal > this->ACTUATION_MAX_VALUE)
                actuateVal = this->ACTUATION_MAX_VALUE;
            else if (actuateVal < this->ACTUATION_MIN_VALUE)
                actuateVal = this->ACTUATION_MIN_VALUE;
            //printf(", %f", actuateVal);
            this->setActuation(int(actuateVal));
            oldError = newError;
            //oldAbsError = absNewError;
            this->actuationControllerlock->unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(int(deltaTime * 1000)));
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

        this->CONTROLLER_I = 0;

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
    //YawActuator.CONTROLLER_I = 0.001;
    YawControllerThread = new std::thread(YawController);
    RollControllerThread = new std::thread(RollController);
    PitchControllerThread = new std::thread(PitchController);
    return 0;
}

int join_RotationalControllers()
{
    YawControllerThread->join();
    RollControllerThread->join();
    PitchControllerThread->join();
    return 0;
}

int destroy_RotationalControllers()
{
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*--------------------------------------------------- VelocityControllers --------------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
    Actuates the drone WRT the Drone's view of the world!
*/

float VelocityControllerErrorScaler(float val) // Use a function
{
    //printf("Yerr: %f\t", val);

    if(val <= 4)
    {
        val /= 1.5;
    }
    return val;
}

class DroneVelocityController_t : public FeedbackController_t
{
public:
    DroneVelocityController_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : FeedbackController_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
    {
        actuateVal = 0;
        currentactuation = 0;
        oldError = 1;
        oldAbsError = 1;
        deltaTime = 1;

        ACTUATION_HALT_VALUE = 0;
        ACTUATION_MAX_VALUE = 60;
        ACTUATION_MIN_VALUE = -60;

        this->MAX_I_BOUNDARY = 100;
        this->CONTROLLER_I = 0.1;
        this->CONTROLLER_D = 10000;
        this->CONTROLLER_P = 0.8;

        ErrorProcessor = VelocityControllerErrorScaler;
    }

    void deployFeedbackControllers()
    {
        //actuatorThread = new std::thread(this->AutoLateralalActuator);
    }
    /*
    void AutoVelocityActuator()
    {
        FeedbackController();
    }*/
};

DroneVelocityController_t X_Vrel_Actuator(setAutoRoll, get_X_VelocityRel, RC_X_MOTION);
DroneVelocityController_t Y_Vrel_Actuator(setAutoPitch, get_Y_VelocityRel, RC_Y_MOTION);
DroneVelocityController_t Z_Vrel_Actuator(setThrottle, get_Z_VelocityRel, THROTTLE);

void X_Vrel_Controllers()
{
    while (true)
    {
        try
        {
            X_Vrel_Actuator.FeedbackController();
        }
        catch (std::exception &e)
        {
            std::cout << "Error in Outermost X_Vrel_Actuator loop!" << e.what();
        }
    }
}

void Y_Vrel_Controllers()
{
    while (true)
    {
        try
        {
            Y_Vrel_Actuator.FeedbackController();
        }
        catch (std::exception &e)
        {
            std::cout << "Error in Outermost Y_Vrel_Actuator loop!" << e.what();
        }
    }
}

void Z_Vrel_Controllers()
{
    while (true)
    {
        try
        {
            Z_Vrel_Actuator.FeedbackController();
        }
        catch (std::exception &e)
        {
            std::cout << "Error in Outermost Z_Vrel_Actuator loop!" << e.what();
        }
    }
}

int init_DroneVelocityControllers()
{
    X_Vrel_ControllerThread = new std::thread(X_Vrel_Controllers);
    Y_Vrel_ControllerThread = new std::thread(Y_Vrel_Controllers);
    //Z_Vrel_ControllerThread = new std::thread(Z_Vrel_Controllers);
    return 0;
}

int join_DroneVelocityControllers()
{
    X_Vrel_ControllerThread->join();
    Y_Vrel_ControllerThread->join();
    //Z_Vrel_ControllerThread->join();
    return 0;
}

int destroy_DroneVelocityControllers()
{
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------------------ */
/*--------------------------------------------- LateralControllers (Positional) --------------------------------------------*/
/* ------------------------------------------------------------------------------------------------------------------------ */

float AltitudeControllerErrorScaler(float val) // Use a function
{
    return -val; //- tanh(val/max_scale) * max_scale;
}

float Planner_X_ControllerErrorScaler(float val) // Use a function
{
    //printf("Xerr: %f\t", val);
    return -val; //- tanh(val/max_scale) * max_scale;
}

float Planner_Y_ControllerErrorScaler(float val) // Use a function
{
    if(val <= 15)
    {
        val /= 5;
    }
    //printf("Yerr: %f\t", val);
    return val; //- tanh(val/max_scale) * max_scale;
}

float LateralControllerErrorScaler(float val) // Use a function
{
    //printf("Yerr: %f\t", val);

    if(val <= 4)
    {
        val /= 1.5;
    }
    return val;
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
        actuateVal = 0;            //127;
        currentactuation = 0;      // 127;
        ACTUATION_HALT_VALUE = 0;  //127;
        ACTUATION_MAX_VALUE = 60;  //255;
        ACTUATION_MIN_VALUE = -60; //0;

        oldError = 1;
        oldAbsError = 1;
        deltaTime = 1;

        this->MAX_I_BOUNDARY = 100;
        this->CONTROLLER_I = 0.0;
        this->CONTROLLER_D = 1000;
        this->CONTROLLER_P = 0.7;

        ErrorProcessor = Planner_X_ControllerErrorScaler;
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
        actuateVal = 0;            //127;
        currentactuation = 0;      // 127;
        ACTUATION_HALT_VALUE = 0;  //127;
        ACTUATION_MAX_VALUE = 60;  //255;
        ACTUATION_MIN_VALUE = -60; //0;

        oldError = 1;
        oldAbsError = 1;
        deltaTime = 1;

        this->MAX_I_BOUNDARY = 100;
        this->CONTROLLER_I = 0.0;
        this->CONTROLLER_D = 10000;
        this->CONTROLLER_P = 0.2;

        ErrorProcessor = Planner_Y_ControllerErrorScaler;
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
        actuateVal = 0;            //127;
        currentactuation = 0;      // 127;
        ACTUATION_HALT_VALUE = 0;  //127;
        ACTUATION_MAX_VALUE = 40;  //255;
        ACTUATION_MIN_VALUE = -40; //0;

        oldError = 1;
        oldAbsError = 1;
        deltaTime = 1;

        this->MAX_I_BOUNDARY = 100;
        this->CONTROLLER_I = 0.1;
        this->CONTROLLER_D = 100000;
        this->CONTROLLER_P = 0.6;

        ErrorProcessor = LateralControllerErrorScaler;
        EscapeFunction = PositionHoldEscapeFunction;
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
        currentactuation = 127;
        ACTUATION_HALT_VALUE = 127;
        ACTUATION_MAX_VALUE = 255;
        ACTUATION_MIN_VALUE = 0;

        oldError = 1;
        oldAbsError = 1;
        deltaTime = 1;

        this->MAX_I_BOUNDARY = 100;
        this->CONTROLLER_I = 0.3;
        this->CONTROLLER_D = 1000000;
        this->CONTROLLER_P = 1.5;

        ErrorProcessor = AltitudeControllerErrorScaler;
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

Planner_X_Controller_t Xrel_Actuator(setAutoPitch, getPathLength, RC_X_MOTION);
Planner_Y_Controller_t Yrel_Actuator(setAutoRoll, getPathDeviation, RC_Y_MOTION);

LateralController_t X_Actuator(setAutoRoll, get_X_Coordinate, RC_X_MOTION);
LateralController_t Y_Actuator(setAutoPitch, get_Y_Coordinate, RC_Y_MOTION);
AltitudeController_t Z_Actuator(setThrottle, getAltitude, THROTTLE);
/*
LateralController_t X_Actuator(set_X_Velocity, get_X_Coordinate, RC_X_MOTION);
LateralController_t Y_Actuator(set_Y_Velocity, get_Y_Coordinate, RC_Y_MOTION);
LateralController_t Z_Actuator(setThrottle, getAltitude, THROTTLE);
*/
void Xrel_Lateral_Controllers()
{
    while (true)
    {
        try
        {
        }
        catch (const std::future_error &e)
        {
            std::cout << "<Xrel_Lateral_Controllers>Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
        }
        catch (std::exception &e)
        {
            std::cout << "Error in Outermost X_Actuator loop!" << e.what();
        }
    }
}

void Yrel_Lateral_Controllers()
{
    while (true)
    {
        try
        {
            Yrel_Actuator.FeedbackController();
        }
        catch (const std::future_error &e)
        {
            std::cout << "<Yrel_Lateral_Controllers>Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
        }
        catch (std::exception &e)
        {
            std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
        }
    }
}

void X_Lateral_Controllers()
{
    while (true)
    {
        try
        {
            X_Actuator.FeedbackController();
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

void Y_Lateral_Controllers()
{
    while (true)
    {
        try
        {
            Y_Actuator.FeedbackController();
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
    Y_Actuator.ErrorProcessor = Planner_Y_ControllerErrorScaler;
    //X_Lateral_ControllerThread = new std::thread(X_Lateral_Controllers);
    //Y_Lateral_ControllerThread = new std::thread(Y_Lateral_Controllers);
    Z_Lateral_ControllerThread = new std::thread(Z_Lateral_Controllers);

    return 0;
}

int join_LateralControllers()
{
    //X_Lateral_ControllerThread->join();
    //Y_Lateral_ControllerThread->join();
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
            FeedbackControl::X_Actuator.FeedbackController();
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
            FeedbackControl::Y_Actuator.FeedbackController();
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

volatile float gazeX;
volatile float gazeY;
volatile float gazeZ;

volatile float pathDestinationX;
volatile float pathDestinationY;
volatile float pathDestinationZ;

volatile float pathStartX;
volatile float pathStartY;
volatile float pathStartZ;

volatile float currentHeading;
std::atomic<float> currentDistance;
volatile float olderDistance;
volatile float initialHeading;

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

float getPathLength()
{
    try
    {
        float sign = 1;

        /*
        float currentActualHeading = getHeadingDegrees();
        if (currentActualHeading > 180)
        {
            currentActualHeading -= 360;
        }
        if (abs(currentActualHeading - currentHeading) >= 90)
        {
            sign = -2;
            //printf("{%f\t%f\t%f}", getHeadingDegrees(), currentHeading, abs(getHeadingDegrees() - currentHeading) );
        } */
        //float dErr = currentDistance - olderDistance;
        float val = currentDistance;
        if(currentDistance <= 10)
        {
            val *= -3;
        }
        
        return val; //sqrt(pow(pathStartX - pathDestinationX, 2) + pow(pathStartY - pathDestinationY, 2));
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
    {
        float dTheta = currentHeading - initialHeading;

        float dev = sinf(dTheta) * currentDistance; // This should give me the deviation
        //printf("{%f\t%f\t%f}\n",dev, dTheta, pathDestinationX);
        return dev;
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

int setLinearPath(GeoPoint_t start, GeoPoint_t destination)
{
    try
    {
        pathDestinationX = destination.x;
        pathDestinationY = destination.y;
        pathDestinationZ = destination.z;

        pathStartX = start.x;
        pathStartY = start.y;
        pathStartZ = start.z;

        // Compute initial heading for future computation

        float dY = pathDestinationY - pathStartY;
        float dX = pathDestinationX - pathStartX;
        float lenXY = sqrt(pow(dX, 2) + pow(dY, 2));
        float cosHeading = dY / lenXY;
        //printf("{%f}", cosHeading);
        float heading = acos(cosHeading);
        heading = copysign(heading, -pathDestinationX + pathStartX);
        initialHeading = (heading * 180) / PI;
    }
    catch (const std::future_error &e)
    {
        std::cout << "<setLinearPath>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
    }
    return 0;
}

void gazeLoop()
{
    try
    {
        int check = 0;
        while (1)
        {
            try
            {
                GeoPoint_t gazeStart = getLocation();
                // Firstly move to the direction to face the destination
                float dY = gazeY - gazeStart.y;
                float dX = gazeX - gazeStart.x;
                float lenXY = sqrt(pow(dX, 2) + pow(dY, 2));
                if (lenXY >= 4) //  it is too close to give any meaningful results
                {
                    check = 0;
                    olderDistance = currentDistance;
                    float cosHeading = dY / lenXY;
                    //printf("{%f}", cosHeading);
                    float heading = acos(cosHeading);
                    heading = copysign(heading, -gazeX + gazeStart.x);
                    float headingDegrees = (heading * 180) / PI;

                    if (isnan(headingDegrees))
                        continue;
                    //printf("[%f\t%f\t%f]", gazeStart.x, gazeStart.y, gazeStart.z);
                    printf("{%f}\n",headingDegrees);
                    //setHeading(headingDegrees);

                    currentHeading = headingDegrees;
                    currentDistance = lenXY;

                    FeedbackControl::YawActuator.actuationControllerlock->lock();
                    FeedbackControl::YawActuator.setIntendedActuation(headingDegrees);
                    FeedbackControl::YawActuator.actuationControllerlock->unlock();
                }
                else 
                {
                    check += 1;
                    if(check >= 10)
                    {
                        printf("Breaking...");
                        return;
                    }
                }

                FeedbackControl::Yrel_Actuator.FeedbackController();
                FeedbackControl::Xrel_Actuator.FeedbackController();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            catch (const std::future_error &e)
            {
                std::cout << "<gazeLoop>Caught a future_error with code \"" << e.code()
                          << "\"\nMessage: \"" << e.what() << "\"\n";
            }
            catch (std::exception &e)
            {
                std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
            }
        }
    }
    catch (const std::future_error &e)
    {
        std::cout << "<gazeLoop Outer>Caught a future_error with code \"" << e.code()
                  << "\"\nMessage: \"" << e.what() << "\"\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Error in Outermost Y_Actuator loop!" << e.what();
    }
}

std::thread *moveThread;

int setGazeOn(GeoPoint_t destination)
{
    try
    {
        gazeX = destination.x;
        gazeY = destination.y;
        gazeZ = destination.z;
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

int launch_ActuationControllers()
{
    try
    {
#if defined(AUTONOMOUS_ACTUATION_CONTROLLERS)
        FeedbackControl::init_RotationalControllers();
        //FeedbackControl::init_DroneVelocityControllers();
        FeedbackControl::init_LateralControllers();
        // Wait for them to join
        FeedbackControl::join_RotationalControllers();
        FeedbackControl::join_LateralControllers();
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