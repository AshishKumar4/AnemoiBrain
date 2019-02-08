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

#include "ControllerInterface.hpp"

#include "FlightControllerInterface.cpp"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------Cli Monitor------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(CLI_MONITOR)

int show_RC = 0, show_PID = 0, show_IMU = 0, show_Wifi = 0, show_armed = 1;

void Channel_ViewRefresh(int threadId)
{
    while (1)
    {
        try
        {
            mtx.lock();
#if defined(MSP_Serial_PROTOCOL)
            if (show_armed)
            {
                if (FlController->isArmed())
                {
                    std::cout << "Armed\t"; // after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
                }
                else
                {
                    std::cout << "Disarmed\t";
                }
            }
#if defined(UPDATE_STATUS_RC)
            if (show_RC)
            {
                msp::msg::Rc rc;
                FlController->client.request(rc);
#if defined(SHOW_STATUS_RC)
                std::cout << rc;
#endif
            }
#endif
#if defined(UPDATE_STATUS_IMU)
            if (show_IMU)
            {
                msp::msg::ImuRaw imu;
                FlController->client.request(imu);
                for (int i = 0; i < 3; i++)
                    IMU_Raw[0][i] = (uint8_t)imu.gyro[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[1][i] = (uint8_t)imu.acc[i];
                for (int i = 0; i < 3; i++)
                    IMU_Raw[2][i] = (uint8_t)imu.magn[i];
#if defined(SHOW_STATUS_IMU)
                std::cout << imu;
#endif
            }
#endif
#if defined(UPDATE_STATUS_PID)
            if (show_PID)
            {
                msp::msg::Pid pid;
                FlController->client.request(pid);
#if defined(SHOW_STATUS_PID)
                std::cout << pid;
#endif
                PID_Raw[0][0] = (uint8_t)pid.roll.P;
                PID_Raw[1][0] = (uint8_t)pid.roll.I;
                PID_Raw[2][0] = (uint8_t)pid.roll.D;
                PID_Raw[0][1] = (uint8_t)pid.pitch.P;
                PID_Raw[1][1] = (uint8_t)pid.pitch.I;
                PID_Raw[2][1] = (uint8_t)pid.pitch.D;
                PID_Raw[0][2] = (uint8_t)pid.yaw.P;
                PID_Raw[1][2] = (uint8_t)pid.yaw.I;
                PID_Raw[2][2] = (uint8_t)pid.yaw.D;
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
                printf("\nHeading: %f", ControllerInterface::getHeading());
#endif
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
            fflush(stdout);
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(CLI_UPDATE_RATE));
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
                      << "\"\nMessage: \"" << e.what() << "\"\n";
            mtx.unlock();
        }
        catch (std::exception &e)
        {
            std::cout << "Error in CLI Monitor " << e.what();
            mtx.unlock();
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

int event_key_other()
{
    return 0;
}

typedef int (*func_t)(); // function pointer
func_t KeyMap[256];

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

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------MSP Data stream forwarding------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MSP_SERIAL_FORWARDING)

//#include "LowLevel/Serial.cpp"

#include "LowLevel/msp/MSP.hpp"
#include "LowLevel/msp/msg_print.hpp"
#include "LowLevel/msp/msp_id.hpp"
#include "LowLevel/msp/FlightController.hpp"

char *bb = new char[4096];

msp::MSP *msp_agent;

MSP_Packet MSP_Agent(char *buf, int size)
{
    try
    {
        int j = 0;
        std::vector<uint8_t> vec((uint8_t *)buf, (uint8_t *)(buf + size));
        /*for (int i = 0; i < size; i++)
        {
            if (buf[i] == '$' && buf[i + 1] == 'M') // MSP Header
            {
                //buf[i+2] contains the direction
                //buf[i+3] contains the size
            }
            printf("[%x]", vec[i]);
        } //*/
        printf("\n Sending...");
        while (msp_agent->write(vec) != 1)
            ;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (msp_agent->hasData() < 1)
        {
            exit(0);
            throw "No Data";
        }

        while (char(msp_agent->read()) != '$')
            ;

        printf("\n Got Data!");

        uint8_t *rdat = (uint8_t *)bb; //malloc(1024); //sz + 1);
        rdat[0] = '$';
        rdat[1] = (uint8_t)msp_agent->read(); //hdr;
        rdat[2] = (uint8_t)msp_agent->read(); //com_state;
        rdat[3] = (uint8_t)msp_agent->read(); //data_size;
        rdat[4] = (uint8_t)msp_agent->read(); //id;

        int i;
        int sz = rdat[3] + 6;
        for (i = 5; i < rdat[3] + 5; i++)
        {
            rdat[i] = (uint8_t)msp_agent->read(); //data[i - 5];
            //printf("{{%d}}", rdat[i]);
        }

        rdat[sz - 1] = (uint8_t)msp_agent->read(); //rcv_crc;
        printf("Size: %d", sz);

        /*for (int j = 0; j < sz; j++)
        {
            printf("<<%x>>", rdat[j]);
        }*/

        return MSP_Packet((char *)rdat, sz);
    }
    catch (std::exception &e)
    {
        printf("\n Some Error");
        //while(1);
        return MSP_Packet(NULL, 0);
    }
}

//std::vector<SerialPort *> defaultPorts;

void Port_Forwarding_Init(int argc, char *argv[])
{
    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
    std::cout << "Opening Port " << device << std::endl;
    msp_agent = new msp::MSP(device, baudrate);
    //defaultPorts.push_back(new SerialPort(device.c_str(), baudrate));
    /*char* buf = new char[4096];
    for(int i = 0; i < 1000; i++)
    {
       // printf(".");
        std::cout<<ReadFromPort(0, buf, 100)<<std::endl;
        printf(buf);
    }
    while(1);*/
}
/*
std::mutex serial_mtx;

int WriteToPort(int portnum, char *buff, int size)
{
    //serial_mtx.lock();
    int a = defaultPorts[portnum]->Write(buff, size);
    //printf("..");
    //serial_mtx.unlock();
    return a;
}

int ReadFromPort(int portnum, char *buff, int size)
{
    //serial_mtx.lock();
    int a = defaultPorts[portnum]->Read(buff, size);
    //serial_mtx.unlock();
    return a;
}*/

#endif

/*
    General Purpose tools
*/

float clamp(float val, float imin, float imax, float omin, float omax)
{
    float aa = omin + (((omax - omin) / (imax - imin)) * (val - imin));
    return aa;
}

namespace ControllerInterface
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ----------------------------------------------General APIs for Control-------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

void setThrottle(int throttle)
{
    unsigned char t = (unsigned char)throttle;
    mtx.lock();
    RC_DATA[THROTTLE] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(throttle, 0);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setPitch(int pitch)
{
    unsigned char t = (unsigned char)pitch;
    mtx.lock();
    RC_DATA[PITCH] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(pitch, 1);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setRoll(int roll)
{
    unsigned char t = (unsigned char)roll;
    mtx.lock();
    RC_DATA[ROLL] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(roll, 2);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setYaw(int yaw)
{
    unsigned char t = (unsigned char)yaw;
    mtx.lock();
    RC_DATA[YAW] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(yaw, 3);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux1(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    // AUX1 to be used for PID Tuning, should set which
    RC_DATA[AUX1] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 4);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux2(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX2] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 5);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux3(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX3] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 6);
#endif
    mtx.unlock();
    //IssueCommand();
}

void setAux4(int val)
{
    unsigned char t = (unsigned char)val;
    mtx.lock();
    RC_DATA[AUX4] = t;
#if defined(SYNCD_TRANSFER)
    sendCommand(val, 7);
#endif
    mtx.unlock();
    //IssueCommand();
}

uint8_t getGyro(int axis)
{
    return IMU_Raw[0][axis];
}

uint8_t getAcc(int axis)
{
    return IMU_Raw[1][axis];
}

uint8_t getMag(int axis)
{
    return IMU_Raw[2][axis];
}

uint8_t getPID_P(int axis)
{
    return PID_Raw[0][axis];
}

uint8_t getPID_I(int axis)
{
    return PID_Raw[1][axis];
}

uint8_t getPID_D(int axis)
{
    return PID_Raw[2][axis];
}

uint8_t getArmStatus(int block)
{
    return 0;
}

/*
    GPS/Barometer/Compass assisted
    All SI Units, Meters, Degrees
*/

namespace // Anonymous Namespace
{
vector3D_t *controlledPosition;
vector3D_t *controlledOrientation;

std::thread *YawControllerThread;
std::thread *RollControllerThread;
std::thread *PitchControllerThread;
std::thread *AltitudeControllerThread;

std::mutex YawControllerlock;
std::mutex PitchControllerlock;
std::mutex RollControllerlock;
std::mutex AltitudeControllerlock;

float Controlled_IntendedYawHeading = 0;
float Controlled_IntendedPitchHeading = 0;
float Controlled_IntendedRollHeading = 0;
float Controlled_IntendedAltitude = 0;

class Actuator_t        // An Abstract class
{
    float IntendedActuation;

  public:
    std::thread *actuatorThread;
    float CONTROLLER_P;
    float CONTROLLER_I;
    float CONTROLLER_D;

    int RC_Channel;

    std::mutex *intentionLock;
    std::mutex *actuationControllerlock;

    std::function<void(int)> setActuation;
    std::function<float(void)> getCurrentStateValues;

    Actuator_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200)
    {
        setActuation = actuatorSet;
        getCurrentStateValues = actuatorGet;
        IntendedActuation = 0;
        CONTROLLER_P = CONSTANT_P;
        CONTROLLER_I = CONSTANT_I;
        CONTROLLER_D = CONSTANT_D;
        intentionLock = new std::mutex();
        actuationControllerlock = new std::mutex();

        RC_Channel = rcChannel;
    }

    int setIntendedActuation(float intention)
    {
        //intentionLock->lock();
        #if defined(ACTUATION_INTENTION_RELATIVE)
        // The Intended actuation should be relative to the current heading
        IntendedActuation = float(int((getHeadingDegrees() + intention)) % 360);
        #else
        IntendedActuation = intention;
        #endif
        //intentionLock->unlock();
        return 0;
    }

    float getIntendedActuation()
    {
        float intention;
        #if defined(ACTUATION_INTENTION_RELATIVE)
        //intentionLock->lock();
        intention = IntendedActuation;
        #else
        intention = IntendedActuation;
        #endif
        //intentionLock->unlock();
        return intention;
    }

    virtual void deployAutoActuators() = 0;
};

/*------------------------------------- RotationalControllers -------------------------------------*/

class RotationActuator_t : public Actuator_t
{
    float errorScale;
    float actuateVal;
    float currentactuation;
    float oldError;
    float oldAbsError;
    float deltaTime;

  public:
    RotationActuator_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : Actuator_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
    {
        errorScale = CONTROLLER_P;
        actuateVal = 127;
        currentactuation = 127;
        oldError = 1;
        oldAbsError = 1;
        deltaTime = 5;
    }

    void deployAutoActuators()
    {
        //actuatorThread = new std::thread(this->AutoRotationalActuator);
    }

    void AutoRotationalActuator()
    {
        try
        {
            this->actuationControllerlock->lock();
            float h = this->getCurrentStateValues();
            float newError = h - this->getIntendedActuation();
            if (newError > 180 || newError < -180)
            {
                newError -= 360;
            }
            float absNewError = std::abs(newError);
            if (absNewError <= 2)
            {
                //direc *= 0.99;
                this->setActuation(127);                                   // Make it not move anymore
                //std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for it to stabilize
                //if (std::abs(this->getCurrentStateValues() - h) < 2)
                //printf("\nDone...%f %f", newError, h);//*/
                this->actuationControllerlock->unlock();
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return;
                //return 1;
            }
            // Our Equation
            float clampedVal = newError * errorScale;                                                                                  //clamp(newError * errorScale, -180, 180, -127, 127);
            actuateVal = ((clampedVal) + ((newError - oldError) / deltaTime) * this->CONTROLLER_D) + RC_MASTER_DATA[this->RC_Channel]; //currentYaw; //(newError/error); // - (newError / oldError)*(absNewError/newError)*2.0 // 2*(oldAbsError - 1*absNewError)

            if (actuateVal > 255)
                actuateVal = 255;
            else if (actuateVal < 0)
                actuateVal = 0;

            //printf("\n{Errors: <%f> %f %f %f [%f]}", h, newError, actuateVal, clampedVal, errorScale);
            this->setActuation(int(actuateVal));
            oldError = newError;
            oldAbsError = absNewError;
            this->actuationControllerlock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(int(deltaTime)));
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
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

RotationActuator_t YawActuator(setYaw, getHeadingDegrees, YAW);
RotationActuator_t RollActuator(setRoll, getRollDegrees, ROLL);
RotationActuator_t PitchActuator(setPitch, getPitchDegrees, PITCH);

void YawController()
{
    while (true)
    {
        try
        {
            YawActuator.AutoRotationalActuator();
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
            RollActuator.AutoRotationalActuator();
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
            PitchActuator.AutoRotationalActuator();
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

void set_X_Motion(int val)
{
    float h = getHeadingDegrees();
    // Equation is, (cos()*setPitch) + (sin()*setRoll)
    float vv = float(val);
    setPitch((vv*cosf(h)));
    setRoll((vv*sinf(h)));
}

void set_Y_Motion(int val)
{
    float h = getHeadingDegrees();
    // Equation is, (cos()*setRoll) + (sin()*setPitch)
    float vv = float(val);
    setPitch((vv*sinf(h)));
    setRoll((vv*cosf(h)));
}

void setAltitude(int val)
{
}

/*------------------------------------- LateralControllers -------------------------------------*/

class LateralActuator_t : public Actuator_t
{
    float errorScale;
    float actuateVal;
    float currentactuation;
    float oldError;
    float oldAbsError;
    float deltaTime;

  public:
    LateralActuator_t(std::function<void(int)> actuatorSet, std::function<float(void)> actuatorGet, int rcChannel, float CONSTANT_P = 0.8, float CONSTANT_I = 1.0, float CONSTANT_D = 200) : Actuator_t(actuatorSet, actuatorGet, rcChannel, CONSTANT_P, CONSTANT_I, CONSTANT_D)
    {
        errorScale = CONTROLLER_P;
        actuateVal = 127;
        currentactuation = 127;
        oldError = 1;
        oldAbsError = 1;
        deltaTime = 5;
    }

    void deployAutoActuators()
    {
        //actuatorThread = new std::thread(this->AutoLateralalActuator);
    }

    void AutoLateralalActuator()
    {
        try
        {
            this->actuationControllerlock->lock();
            float h = this->getCurrentStateValues();
            float newError = h - this->getIntendedActuation();
            float absNewError = std::abs(newError);
            if (absNewError <= 2)
            {
                //direc *= 0.99;
                this->setActuation(127);                                   // Make it not move anymore
                //std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Wait for it to stabilize
                //if (std::abs(this->getCurrentStateValues() - h) < 2)
                    printf("\nDone...%f %f", newError, h);//*/
                this->actuationControllerlock->unlock();
                //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return;
                //return 1;
            }
            // Our Equation
            float clampedVal = newError * errorScale;                                                                                  //clamp(newError * errorScale, -180, 180, -127, 127);
            actuateVal = ((clampedVal) + ((newError - oldError) / deltaTime) * this->CONTROLLER_D) + RC_MASTER_DATA[this->RC_Channel]; //currentYaw; //(newError/error); // - (newError / oldError)*(absNewError/newError)*2.0 // 2*(oldAbsError - 1*absNewError)

            if (actuateVal > 255)
                actuateVal = 255;
            else if (actuateVal < 0)
                actuateVal = 0;

            //printf("\n{Errors: <%f> %f %f %f [%f]}", h, newError, actuateVal, clampedVal, errorScale);
            this->setActuation(int(actuateVal));
            oldError = newError;
            oldAbsError = absNewError;
            this->actuationControllerlock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(int(deltaTime)));
        }
        catch (const std::future_error &e)
        {
            std::cout << "Caught a future_error with code \"" << e.code()
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

LateralActuator_t X_Actuator(set_X_Motion, get_X_Coordinate, RC_X_MOTION);
LateralActuator_t Y_Actuator(set_Y_Motion, get_Y_Coordinate, RC_Y_MOTION);
LateralActuator_t Z_Actuator(setThrottle, getAltitude, THROTTLE);

/*
    Back to the APIs
*/

vector3D_t eulerFromQuaternion(quaternion_t orien)
{
    vector3D_t oo;
    /*
        Quaternion to Euler
    */
    //std::cout << ">>>" << orien << "<<<" << std::endl;
    double heading = 0, roll, pitch, yaw;
    double ysqr = orien.y() * orien.y();

    // roll (x-axis rotation)
    double t0 = +2.0f * (orien.w() * orien.x() + orien.y() * orien.z());
    double t1 = +1.0f - 2.0f * (orien.x() * orien.x() + ysqr);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0f * (orien.w() * orien.y() - orien.z() * orien.x());
    t2 = ((t2 > 1.0f) ? 1.0f : t2);
    t2 = ((t2 < -1.0f) ? -1.0f : t2);
    pitch = std::asin(t2);

    // yaw (z-axis rotation)
    double t3 = +2.0f * (orien.w() * orien.z() + orien.x() * orien.y());
    double t4 = +1.0f - 2.0f * (ysqr + orien.z() * orien.z());
    yaw = std::atan2(t3, t4);
    //heading = yaw;
    //printf("->Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
    //printf("->Heading: { %f %f}", orien.z(), orien.w());
    oo.x = pitch;
    oo.y = roll;
    oo.z = yaw; // // 0, 360);
    return oo;
}

int init_RotationalControllers()
{
    YawControllerThread = new std::thread(YawController);
    return 0;
}

int destroy_RotationalControllers()
{
    YawControllerThread->join();
    return 0;
}

int init_LateralControllers()
{
    //YawControllerThread = new std::thread(YawController);
    return 0;
}

int destroy_LateralControllers()
{
    //YawControllerThread->join();
    return 0;
}

/* Our Anonymous Namespace ends here, Autonomous APIs Below */

int init_ActuationControllers()
{
#if defined(AUTONOMOUS_ACTUATION_CONTROLLERS)   
    init_RotationalControllers();
    //init_LateralControllers();
#endif
    return 0;
}

int destroy_ActuationControllers()
{
    destroy_RotationalControllers();
    destroy_LateralControllers();
    return 0;
}

int pauseControllers()
{
    YawActuator.actuationControllerlock->lock();
    return 1;
}

int resumeControllers()
{
    YawActuator.actuationControllerlock->unlock();
    return 0;
}

} // namespace


quaternion_t getOrientationQuaternion()
{
#if defined(MODE_AIRSIM)
    auto orien = client.getMultirotorState().getOrientation();
    return quaternion_t(orien.w(), orien.x(), orien.y(), orien.z());
#else
    return quaternion_t(1, 0, 0, 0);
#endif
}

vector3D_t getOrientation() // Returns Euler angle orientation
{
    return eulerFromQuaternion(getOrientationQuaternion());
}

float getYaw() // Gives in Radians
{
    float h = getOrientation().z;
    return h;
}

float getRoll() // Gives in Radians
{
    float h = getOrientation().y;
    return h;
}

float getPitch() // Gives in Radians
{
    float h = getOrientation().x;
    return h;
}

/*
    This is our convention, counter clockwise, 0 to 360 in every direction
*/

float getConventionalDegrees(float rads)
{
    // 0 -> North
    // 90 -> east
    // 180 -> south
    // 270 ->west
    float h = clamp(rads, -3.14159265358979323846, 3.14159265358979323846, 180, -180);
    if (h < 0)
    {
        h = 360 + h;
    }
    return h;
}

float getYawDegrees()
{
    return getConventionalDegrees(getYaw());
}

float getRollDegrees()
{
    return getConventionalDegrees(getRoll());
}

float getPitchDegrees()
{
    return getConventionalDegrees(getPitch());
}

float get_X_Coordinate()
{
    return 0;
}

float get_Y_Coordinate()
{
    return 0;
}

float getAltitude()
{
    return 0;
}

float getHeadingDegrees() // Gives in Degrees
{
    return getYawDegrees();
}

float getHeading()
{
    return getHeadingDegrees();
}

/*
    A Little Higher Level APIs
*/

int setHeading(float heading)
{
    return setAutoYaw(heading);
}

int testHeading(std::vector<std::string> test)
{
    std::cout << "Testing Auto Heading feature...";
    return setHeading(90);
}

int setAutoYaw(float heading)
{
    YawActuator.actuationControllerlock->lock();
    YawActuator.setIntendedActuation(heading);
    YawActuator.actuationControllerlock->unlock();
    return 0;
}

int setAutoRoll(float heading)
{
    RollActuator.actuationControllerlock->lock();
    RollActuator.setIntendedActuation(heading);
    RollActuator.actuationControllerlock->unlock();
    return 0;
}

int setAutoPitch(float heading)
{
    PitchActuator.actuationControllerlock->lock();
    PitchActuator.setIntendedActuation(heading);
    PitchActuator.actuationControllerlock->unlock();
    return 0;
}

vector3D_t getVelocity()
{
    vector3D_t vel;
    return vel;
}

vector3D_t getPosition()
{
    vector3D_t pos;
    return pos;
}

void setAltitude(float altitude)
{
}

void takeOff(float altitude)
{
    setAltitude(5);
}

void holdPosition()
{
}

void moveToRelativePosition(float x, float y, float z)
{
}

/*
    High Level APIs 
*/

int autoNavPID(vector3D_t start, vector3D_t destination, float maxAltitude)
{
    // Firstly move to the direction to face the destination

    // Then Throttle Up to maintain an altitude.
    return 0;
}

std::vector<vector3D_t> destinationBuffer;

int setDestination(vector3D_t position, bool start_now)
{
    if (start_now)
        autoNavPID(getPosition(), position);
    return 0;
}

int returnToHome()
{
    return 0;
}

/*
    Some other Mechanisms
*/

int unusedAPIhandler(std::vector<std::string> test)
{
    std::cout << "\nOops! Seems someone sent me some wrong code!";
    return 1;
}

void RemoteAPI_Invoker(int code, int count)
{
    std::vector<std::string> aa;
    printf("<<%d, %d>>", code, count);
    std::cout << API_ProcedureInvokeTable[code](aa);
}

void FailSafeMechanism()
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
        for (int i = a; i > 0; i--)
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
        // Disarm and send FailSafe!
        FailSafeTrigger = true;
        setAux1(51); // Trigger Failsafe
        std::cout << "\nFailSafe Locked!";
        mtx.lock(); // Grab the lock and don't release until the fault is fixed
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 2!" << e.what();
    }
}

void ResumeHandler()
{
#if defined(MSP_Serial_PROTOCOL)
    try
    {
        failsafe.lock();
        FaultManaged = true;
        failsafe.unlock();
        FailSafeThread->join();
        if (FailSafeTrigger)
        {
            mtx.unlock(); // Grab the lock and don't release until the fault is fixed
            FailSafeTrigger = false;
        }
        std::cout << "Fault Resumed and Managed!\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 3!" << e.what();
    }
#endif
}

void FaultHandler()
{
#if defined(MSP_Serial_PROTOCOL)
    try
    {
        std::cout << "Fault Occured!\nTriggering FailSafe!!!";
        FaultManaged = false;
        FailSafeThread = new std::thread(FailSafeMechanism);
    }
    catch (std::exception &e)
    {
        std::cout << "Some More Error 1!" << e.what();
    }
#endif
}

int ControllerInterface_init(int argc, char **argv)
{
    printf("\n Initializing Flight Controller Interface...");
    //int fd = wiringPiSPISetup(0, SPI_IOC_WR_MAX_SPEED_HZ);//SPI_init("/dev/spidev0.0");
    Raw_Init(argc, argv);
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

    //SPI_handshake();
    //IssueCommand();

    for (int i = 0; i < 256; i++)
    {
        API_ProcedureInvokeTable[i] = unusedAPIhandler;
    }
    API_ProcedureInvokeTable[111] = testHeading;

    init_ActuationControllers();

#if defined(CLI_MONITOR)
    std::thread *chnl_refresh = new std::thread(Channel_ViewRefresh, 0);
    for (int i = 0; i < 256; i++)
        KeyMap[i] = event_key_other;
    KeyMap['q'] = event_key_q; // Show RC Values
    KeyMap['w'] = event_key_w; // show PID Values
    KeyMap['e'] = event_key_e; // show IMU Data
    KeyMap['r'] = event_key_r; // show Wifi Strength
    KeyMap['A'] = event_key_A; // show Armed
    std::thread *keyboard_handler = new std::thread(Keyboard_handler, 2);
#endif
#if defined(UPDATER_THREAD)
    std::thread *chnl_update = new std::thread(Channel_Updater, 1);
#endif

    return 0;
}

} // namespace ControllerInterface
