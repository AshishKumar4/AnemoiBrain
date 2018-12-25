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

namespace Onboard
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

int opt = 1;

const char HANDSHAKE_IN_MSG[] = "Hello Gardien!";
const char HANDSHAKE_OUT_MSG[] = "Hello Overloard!";

void AbstractServer::AddChannels(func_iii_t function) // Should be called once
{
    server_fd.push_back(0);
    addresses.push_back(NULL);
    ChannelOperators.push_back(function);
}

void AbstractServer::CreateChannels(func_iii_t function) // Should be called once
{
    AddChannels(function);
    this->ListenerThreads.push_back(new std::thread(&AbstractServer::ChannelListeners, ChannelOperators.size() - 1, this));
}

int AbstractServer::LaunchThreads()
{
    if (!ListenerThreads.size())
    {
        for (int i = 0; i < ChannelOperators.size(); i++)
        {
            this->ListenerThreads.push_back(new std::thread(&AbstractServer::ChannelListeners, i, this));
        }
    }

    for (int i = 0; i < ChannelOperators.size(); i++)
    {
        ListenerThreads[i]->join();
    }
    //std::cout << "Broken Pipe, Waiting for incoming Connections...";
    return 0;
}

AbstractServer::AbstractServer(int portBase)
{
    PORT_BASE = portBase;
}
AbstractServer::~AbstractServer()
{
    // TODO: Release all the sockets here
}

void AbstractServer::ChannelListeners(int i, AbstractServer* thisObj)
{
    int PORT = i + thisObj->PORT_BASE;
    thisObj->SetupChannel(PORT, i);
    int new_socket;
    ssize_t valread = 0;
back:
    thisObj->smtx.lock();
    int sfd = thisObj->server_fd[i];
    struct sockaddr_in *address = thisObj->addresses[i];
    char *buff = new char[4096];

    std::cout << "\n\nServer Initialized at port " << PORT << " Successfully...";
    if (listen(sfd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    std::cout << "\nwaiting for incoming Connections...";
    thisObj->smtx.unlock();
    while (1)
    {
        int addrlen = sizeof(struct sockaddr);
        if ((new_socket = accept(sfd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
        {
            //perror("accept");
            //exit(EXIT_FAILURE);
            printf("accept");
            delete [] buff;
            goto back;
        }
        std::cout << "\nGot an incoming request...\n";
        try
        {
            thisObj->smtx.lock();
            valread = read(new_socket, buff, 1024);
            if (valread == 0)
                continue;

            if (strncmp(buff, HANDSHAKE_IN_MSG, strlen(HANDSHAKE_IN_MSG)))
            {
                std::cout << "Overloard Could not establish Connection / Handshake Failure...\n";
                continue;
            }
            else
            {
                send(new_socket, HANDSHAKE_OUT_MSG, strlen(HANDSHAKE_OUT_MSG), 0);
                std::cout << "Overloard Connected Successfully...\n";
            }
            thisObj->smtx.unlock();
        }
        catch (std::exception &e)
        {
            std::cout << "Some ERROR!!!" << e.what() << "\n";
        }
        try
        {
            while (1)
            {
                if (thisObj->ChannelOperators[i](i, new_socket))
                    break;
            }
        }
        catch (std::exception &e)
        {
            std::cout << "Broken Pipe, Waiting for incoming Connections...";
        }
    }
}

int AbstractServer::SetupChannel(int port, int channel) // This would create a port for a particular channel
{
    smtx.lock();
    int sfd;
    if (server_fd[channel])
    {
        //delete server_fd[channel];
        //free((void*)server_fd[channel]);
        //delete addresses[channel];
    }
    struct sockaddr_in *address = new struct sockaddr_in;
    // Creating socket file descriptor
    if ((sfd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address->sin_family = AF_INET;
    address->sin_addr.s_addr = INADDR_ANY;
    address->sin_port = htons(port);

    // Forcefully attaching socket to the given port
    //bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in));
    if (::bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    try
    {
        // TODO: Free up the existing sockets
        server_fd[channel] = sfd;
        //delete addresses[channel];
        addresses[channel] = address;
    }
    catch (...)
    {
        perror("Error while Saving the Socket Memory Structures");
        exit(EXIT_FAILURE);
    }
    std::cout << "Socket for port " << port << " Created Successfully!" << std::endl;
    smtx.unlock();
    return 0;
}
} // namespace Onboard
