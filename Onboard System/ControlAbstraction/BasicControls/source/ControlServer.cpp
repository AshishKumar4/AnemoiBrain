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
#include "BasicControls.h"

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
namespace Controls
{

int opt = 1;

func_vi_t CHANNEL_HANDLER_TABLES[] = {&setThrottle, &setPitch, &setRoll, &setYaw, &setAux1, &setAux2};
std::string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2"};

char **ControlChannelBuffer = (char **)malloc(sizeof(char *) * 8);

int ControlListeners(int i, int fd)
{
    try
    {
        memset(ControlChannelBuffer[i], 0, 4096);
        int valread = read(fd, ControlChannelBuffer[i], 4096);
        if (valread == 0 || valread == -1)
            return 1;
#if defined(STREAM_PROTOCOL_1)
        std::string parsed, cmd(ControlChannelBuffer[i]);
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

            //std::cout << endl << par1 << " " << "<" << val.length() << ">" << val << " (" << atoi(val.c_str()) << ") " << par2 << "\n";

            // We Now issue our command
            //printf("\n [%s] Command Issued ", CHANNEL_NAME_TABLES[i].c_str());

#ifndef DRONELESS_LOCAL_TEST
            CHANNEL_HANDLER_TABLES[i](atoi(val.c_str()));
#endif
        }
#elif defined(STREAM_PROTOCOL_2)
        for (int j = 0; j < valread; j++)
        {
            CHANNEL_HANDLER_TABLES[i](int(ControlChannelBuffer[i][j]));
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
} // namespace Controls

#if defined(MSP_SERIAL_FORWARDING)

namespace SerialForwarding
{
char *buff = new char[8192];

int ControlListeners(int i, int fd)
{
    try
    {
        memset(buff, 0, 4096);
        int valread = read(fd, buff, 4096);
        if (valread == 0 || valread == -1)
            return 1;

        
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR!!!" << e.what() << "\n";
        return 1;
    }
    return 0;
}
} // namespace SerialForwarding

#endif

} // namespace Onboard

int main(int argc, char const *argv[])
{
    char *buff = new char[1024];
#ifndef DRONELESS_LOCAL_TEST
    BasicControls_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    Onboard::AbstractServer ControlServer(8400);
    for (int i = 0; i < 6; i++)
    {
        Onboard::Controls::ControlChannelBuffer[i] = new char[4096];
        ControlServer.AddChannels(Onboard::Controls::ControlListeners);
    }
    ControlServer.LaunchThreads();
    while (1)
        std::cout << "Some Error!";
    ;
    return 0;
}
