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

#include "ControlServer.hpp"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

namespace Onboard
{
namespace Controls
{

int opt = 1;

func_vi_t CHANNEL_HANDLER_TABLES[] = {&ControllerInterface::setThrottle, &ControllerInterface::setPitch, &ControllerInterface::setRoll, &ControllerInterface::setYaw, &ControllerInterface::setAux1, &ControllerInterface::setAux2, &ControllerInterface::setAux3, &ControllerInterface::setAux4};
std::string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2", "aux3", "aux4"};

char **ControlChannelBuffer = (char **)malloc(sizeof(char *) * 12);

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

int ControlExceptionHandler()
{
    ControllerInterface::FaultHandler();
    return 1;
}

int ControlResumeHandler()
{
    ControllerInterface::ResumeHandler();
    return 1;
}

int RemoteAPI_Listener(int i, int fd)
{
    try 
    {
        memset(ControlChannelBuffer[i], 0, 4096);
        printf("\nInsiteRAPI");
        int valread = read(fd, ControlChannelBuffer[i], 4096);
        if (valread == 0 || valread == -1)
            return 1;
        // Implement a protocol over here to pass on codes 

        // for now, lets just test if it works 
        //std::vector<std::string> test;
        printf("\nRAPI CALLED!!!");
        for(int j = 0; j < valread; j++)
        {
            ControllerInterface::RemoteAPI_Invoker((int)ControlChannelBuffer[i][j], j);
        }
        printf("\n Exiting...");
    }
    catch(std::exception &e)
    {
        std::cout<<"ERROR!!!";
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
            int numVal = atoi(val.c_str());
#ifndef DRONELESS_LOCAL_TEST
            ControllerInterface::RC_MASTER_DATA[i] = uint8_t(numVal);
            CHANNEL_HANDLER_TABLES[i](numVal);
#endif
        }
#elif defined(STREAM_PROTOCOL_2)
        for (int j = 0; j < valread; j++)
        {
            CHANNEL_HANDLER_TABLES[i](int(ControlChannelBuffer[i][j]));
            ControllerInterface::RC_MASTER_DATA[i] = uint8_t(ControlChannelBuffer[i][j]);
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
        //ControllerInterface::WriteToPort(0, buff, valread);
        for (int i = 0; i < 1000; i++)
        {
            int valread = ControllerInterface::ReadFromPort(0, buff2, 32);
            printf("{%s}", buff2);
            write(fd, buff2, valread);
            memset(buff2, 0, strlen(buff2));
        }
        /*std::this_thread::sleep_for(std::chrono::miliseconds(10));
        int valread = ControllerInterface::ReadFromPort(0, buff2, 32);
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
            int valread = ControllerInterface::ReadFromPort(0, buff2, 32);
            printf("{%s}", buff2);
            write(fd, buff2, valread);
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

int ControlServer_init(int argc, char **argv)
{
    int portBase = (argc > 1) ? stoi(std::string(argv[3])) : 8400;
    Onboard::AbstractServer ControlServer(portBase);
    for (int i = 0; i < 8; i++)
    {
        Onboard::Controls::ControlChannelBuffer[i] = new char[4096];
        ControlServer.AddChannels(i, Onboard::Controls::ControlListeners, Onboard::Controls::ControlHandshake);
    }
    Onboard::Controls::ControlChannelBuffer[8] = new char[4096];
    ControlServer.AddChannels(8, Onboard::Controls::RemoteAPI_Listener, Onboard::Controls::ControlHandshake);

    ControlServer.ExceptionHandler = Onboard::Controls::ControlExceptionHandler;
    ControlServer.ResumeHandler = Onboard::Controls::ControlResumeHandler;
    ControlServer.LaunchThreads(); //*/

#if defined(MSP_REMOTE_TWEAKS)

    Onboard::AbstractServer RemoteTweakerServer(portBase + 200);
    RemoteTweakerServer.CreateChannels(0, Onboard::RemoteTweaker::RemotePIDChange, Onboard::RemoteTweaker::Handshake);
    RemoteTweakerServer.JoinThreads();

#endif

#if defined(MSP_SERIAL_FORWARDING)
    Onboard::AbstractServer SerialForwardServer(portBase + 100);
    Onboard::SerialForwarding::sock_locks.push_back(new std::mutex);
    SerialForwardServer.CreateChannels(0, Onboard::SerialForwarding::MSP_Forward, Onboard::SerialForwarding::Handshake);
    SerialForwardServer.JoinThreads();
#endif //*/
    ControlServer.JoinThreads();
    return 0;
}

} // namespace Onboard

