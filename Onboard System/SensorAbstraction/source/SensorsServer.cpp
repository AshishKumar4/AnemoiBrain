#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>

#include "AbstractServer.hpp"
#include "ControllerInterface.hpp"
#include "Sensors.h"

#include "SensorsServer.hpp"

namespace Onboard
{
namespace Sensors
{
typedef uint8_t (*func_i8i_t)(int);              // function pointer
func_i8i_t CHANNEL_HANDLER_TABLES[] = {&ControllerInterface::getGyro, &ControllerInterface::getAcc, &ControllerInterface::getMag, &ControllerInterface::getPID_P, &ControllerInterface::getPID_I, &ControllerInterface::getPID_D, &ControllerInterface::getArmStatus};
std::string CHANNEL_NAME_TABLES[] = {"gyro", "acc", "mag", "P", "I", "D", "arm"};

char **SensorsChannelBuffer = (char **)malloc(sizeof(char *) * 8);

int SensorExceptionHandler()
{
    ControllerInterface::FaultHandler();
    return 1;
}

int SensorResumeHandler()
{
    ControllerInterface::ResumeHandler();
    return 1;
}

int SensorsHandshake(int i, int fd)
{
    try
    {
        //thisObj->smtx.lock();
        int valread = read(fd, SensorsChannelBuffer[i], 1024);
        if (valread == 0)
            return 1;

        if (strncmp(SensorsChannelBuffer[i], HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG)))
        {
            std::cout << "Overloard Could not establish Connection / Handshake Failure...\n";
            return 1;
        }
        else
        {
            send(fd, HANDSHAKE_OUT_MSG, strlen(HANDSHAKE_OUT_MSG), 0);
            std::cout << "Overloard Connected Successfully...\n";
        }
        //thisObj->smtx.unlock();
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR in Handshake!!!" << e.what() << "\n";
        return 1;
    }
    return 0;
}

int SensorsListeners(int i, int fd)
{
    try
    {
        // We are using the Stream Protocol 2 by default, Implement Protocol 1 if required, with ControlServer as reference
        SensorsChannelBuffer[i][0] = CHANNEL_HANDLER_TABLES[i](0);
        write(fd, SensorsChannelBuffer[i], 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (std::exception &e)
    {
        printf("\n{Some Error #1}");
        std::cout<<e.what();
        return 1;
    }
    return 0;
}

} // namespace Sensors

int SensorsServer_init(int argc, char **argv)
{
    Onboard::AbstractServer SensorsServer(8300);
    for (int i = 0; i < 7; i++) // gyro, acc, mag, P, I, D, arm_status
    {
        Onboard::Sensors::SensorsChannelBuffer[i] = new char[2];
        SensorsServer.AddChannels(i, Onboard::Sensors::SensorsListeners, Onboard::Sensors::SensorsHandshake);
    }
    SensorsServer.ExceptionHandler = Onboard::Sensors::SensorExceptionHandler;
    SensorsServer.ResumeHandler = Onboard::Sensors::SensorResumeHandler;
    SensorsServer.LaunchThreads();
    while (1)
        std::cout << "Some Error!";
    ;
    return 0;
}
} // namespace Onboard