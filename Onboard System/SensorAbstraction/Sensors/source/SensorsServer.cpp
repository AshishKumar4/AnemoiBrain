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
#include "Sensors.h"

namespace Onboard
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------------------Some Configurations--------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/*
  STREAM_PROTOCOL_1 Refers to conventional failproof streaming of values as packets, encapsulated and later stripped off on the other hand 
  STREAM_PROTOCOL_2 Refers to newer, faster and lightweight but simplest streaming, stream of simple bytes. This isn't failproof and errors 
                    may be entroduced.
*/
//#define STREAM_PROTOCOL_1 // FailProof, Encapsulate
#define STREAM_PROTOCOL_2 // Simple Byte Stream, Faster and lightweight

//#define DRONELESS_LOCAL_TEST

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------Permanent Configurations------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(STREAM_PROTOCOL_1)
#undef STREAM_PROTOCOL_2
#elif defined(STREAM_PROTOCOL_2)
#undef STREAM_PROTOCOL_1
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */
namespace Sensors
{

int opt = 1;

func_vi_t CHANNEL_HANDLER_TABLES[] = {&setThrottle, &setPitch, &setRoll, &setYaw, &setAux1, &setAux2};
std::string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2"};

char **SensorsChannelBuffer = (char **)malloc(sizeof(char *) * 8);

int SensorsListeners(int i, int fd)
{
    try
    {
        memset(SensorsChannelBuffer[i], 0, 4096);
        int valread = read(fd, SensorsChannelBuffer[i], 4096);
        if (valread == 0 || valread == -1)
            return 1;
#if defined(STREAM_PROTOCOL_1)
        std::string parsed, cmd(SensorsChannelBuffer[i]);
        //std::cout << cmd << endl;

        /*
            Format of the input command ->
            .[:x:].
        */
        std::stringstream input_stringstream(cmd);
        // Split the input line into several [:x:]
        while (std::getline(input_stringstream, parsed, '.'))
        {
            if (!parsed.length())
                continue;
            //std::cout << "Parsed: [" << parsed << "],\n";
            std::string par1, par2, val;
            std::stringstream parsed_stream(parsed);
            std::getline(parsed_stream, par1, ':');
            std::getline(parsed_stream, val, ':'); // Extract the x
            
#ifndef DRONELESS_LOCAL_TEST
            CHANNEL_HANDLER_TABLES[i](atoi(val.c_str()));
#endif
        }
#elif defined(STREAM_PROTOCOL_2)
        for (int j = 0; j < valread; j++)
        {
            CHANNEL_HANDLER_TABLES[i](int(SensorsChannelBuffer[i][j]));
        }
#endif
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR!!!" << e.what() << "\n";
        return 1;
    }
    return 0;
}
} // namespace Sensors
} // namespace Onboard

int main(int argc, char const *argv[])
{
    char *buff = new char[1024];
#ifndef DRONELESS_LOCAL_TEST
    BasicControls_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    Onboard::AbstractServer SensorsServer(8400);
    for (int i = 0; i < 6; i++)
    {
        Onboard::Sensors::SensorsChannelBuffer[i] = new char[4096];
        SensorsServer.AddChannels(Onboard::Sensors::SensorsListeners);
    }
    ControlServer.LaunchThreads();
    while (1)
        std::cout << "Some Error!";
    ;
    return 0;
}
