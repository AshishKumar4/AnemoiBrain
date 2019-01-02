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

func_vi_t CHANNEL_HANDLER_TABLES[] = {&BasicControls::setThrottle, &BasicControls::setPitch, &BasicControls::setRoll, &BasicControls::setYaw, &BasicControls::setAux1, &BasicControls::setAux2};
std::string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2"};

char **ControlChannelBuffer = (char **)malloc(sizeof(char *) * 8);

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";

int ControlHandshake(int i, int fd)
{
    try
    {
        //thisObj->smtx.lock();
        int valread = read(fd, ControlChannelBuffer[i], 1024);
        if (valread == 0)
            return 1;

        if (strncmp(ControlChannelBuffer[i], HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG)))
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

#include "LowLevel/Serial.hpp"

namespace SerialForwarding
{
char *buff = new char[8192];
char *buff2 = new char[8192];

std::vector<std::mutex *> sock_locks;

void InitializerServer(int i)
{
}

int ControlListeners(int i, int fd) // Listens to Socket, Writes to Serial
{
    try
    {
        memset(buff, 0, 4096);
        int valread = read(fd, buff, 4096);
        // Grab a lock!
        sock_locks[0]->lock();
        if (valread == 0 || valread == -1)
            return 1;

        // Write this onto the serial port
        printf("\n(%s)", buff);
        //BasicControls::WriteToPort(0, buff, valread);
        for (int i = 0; i < 1000; i++)
        {
            int valread = 0; //BasicControls::ReadFromPort(0, buff2, 32);
            printf("{%s}", buff2);
            memset(buff2, 0, strlen(buff2));
        }
        /*std::this_thread::sleep_for(std::chrono::miliseconds(10));
        int valread = BasicControls::ReadFromPort(0, buff2, 32);
        printf("%s", buff2);
        write(fd, buff2, valread);
        sock_locks[0]->unlock();*/

        sock_locks[0]->unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR in Serial Forwarding Write!!!" << e.what() << "\n";
        sock_locks[i]->unlock();
        return 1;
    }
    return 0;
}

int ControlWriters(int i, int fd) // Listens to Serial, Writes to Socket
{
    try
    {
        memset(buff2, 0, 4096);
        // Grab a lock!
        //sock_locks[0]->lock();
        //printf("\n$>>\t");
        // Write this onto the serial port
        for (int i = 0; i < 1000; i++)
        {
            int valread = 0; //BasicControls::ReadFromPort(0, buff2, 32);
            printf("{%s}", buff2);
            memset(buff2, 0, strlen(buff2));
        }
        //sock_locks[0]->unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR in Serial Forwarding Read!!!" << e.what() << "\n";
        sock_locks[0]->unlock();
        return 1;
    }
    return 0;
}

int MSP_Forward(int i, int fd)
{
    try
    {
        memset(buff2, 0, 4096);
        int valread = read(fd, buff, 4096);
        // Grab a lock!
        // sock_locks[0]->lock();
        if (valread == 0 || valread == -1)
            return 1;
        printf("\n<%s>", buff);
        MSP_Packet msp_packet = MSP_Agent(buff, valread);
        write(fd, msp_packet.buf, msp_packet.size);
        std::cout << std::endl;
        /*for (int j = 0; j < msp_packet.size; j++)
        {
            printf("<<%x>>", msp_packet.buf[j]);
        }*/
        //sock_locks[0]->unlock();
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR in Serial!!!" << e.what() << "\n";
        //sock_locks[0]->unlock();
        return 1;
    }
    return 0;
}

int Handshake(int i, int j)
{
    return 0;
}
} // namespace SerialForwarding

namespace RemoteTweaker
{
int RemotePIDChange(int i, int fd)
{
    try
    {
        memset(buff2, 0, 4096);
        int valread = read(fd, buff, 4096);
        if (valread == 0 || valread == -1)
            return 1;
        printf("\n<%s>", buff);
        
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR in Serial!!!" << e.what() << "\n";
        //sock_locks[0]->unlock();
        return 1;
    }
    return 0;
}

int Handshake(int i, int j)
{
    return 0;
}
} // namespace RemoteTweaker

#endif

} // namespace Onboard

int main(int argc, char const *argv[])
{
#ifndef DRONELESS_LOCAL_TEST
    BasicControls_init(argc, (char **)argv); // Maybe lower levels can make use of command line args
#endif
    Onboard::AbstractServer ControlServer(8400);
    for (int i = 0; i < 6; i++)
    {
        Onboard::Controls::ControlChannelBuffer[i] = new char[4096];
        ControlServer.AddChannels(i, Onboard::Controls::ControlListeners, Onboard::Controls::ControlHandshake);
    }
    ControlServer.LaunchThreads(); //*/

#if defined(MSP_REMOTE_TWEAKS)

    Onboard::AbstractServer RemoteTweakerServer(8600);
    RemoteTweakerServer.CreateChannels(0, Onboard::RemoteTweaker::RemotePIDChange, Onboard::RemoteTweaker::Handshake);
    RemoteTweakerServer.JoinThreads();

#endif

#if defined(MSP_SERIAL_FORWARDING)
    Onboard::AbstractServer SerialForwardServer(8500);

    Onboard::SerialForwarding::sock_locks.push_back(new std::mutex);

    SerialForwardServer.CreateChannels(0, Onboard::SerialForwarding::MSP_Forward, Onboard::SerialForwarding::Handshake);

    SerialForwardServer.JoinThreads();
#endif //*/
    ControlServer.JoinThreads();
    return 0;
}
