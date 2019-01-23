#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>

#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>


#include "AbstractServer.hpp"

namespace Onboard
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

int opt = 1;

void AbstractServer::AddChannels(int index, func_iii_t function, func_iii_t initializer, int subchannel) // Should be called once
{
    if (ChannelOperators.size() <= index)
    {
        std::vector<func_iii_t> vec;
        vec.push_back(function);
        ChannelOperators.push_back(vec);
        ChannelInitializers.push_back(initializer);
        server_fd.push_back(0);
        addresses.push_back(NULL);
    }
    else
    {
        if (ChannelOperators[index].size() <= subchannel)
        {
            ChannelOperators[index].push_back(function);
        }
        else
        {
            ChannelOperators[index][subchannel] = function;
        }
        ChannelInitializers[index] = initializer;
    }
}

void AbstractServer::CreateChannels(int index, func_iii_t function, func_iii_t initializer, int subchannel, bool initPort) // Should be called once
{
    AddChannels(index, function, initializer, subchannel);
    if (initPort)
    {
        SetupChannel(PORT_BASE + index, index);
    }
    this->ListenerThreads.push_back(new std::thread(&AbstractServer::ChannelListener, index, this));
}

int AbstractServer::LaunchThreads(bool initPort)
{
    if (!ListenerThreads.size())
    {
        for (int i = 0; i < ChannelOperators.size(); i++)
        {
            if (initPort)
                SetupChannel(PORT_BASE + i, i);
            this->ListenerThreads.push_back(new std::thread(&AbstractServer::ChannelListener, i, this));
        }
    }
    return 0;
}

int AbstractServer::JoinThreads()
{
    for (int i = 0; i < ChannelOperators.size(); i++)
    {
        ListenerThreads[i]->join();
    }
    return 0;
}

AbstractServer::AbstractServer(int portBase)
{
    PORT_BASE = portBase;
}

AbstractServer::AbstractServer(AbstractServer *obj)
{
    PORT_BASE = obj->PORT_BASE;
    // Make this up
}

AbstractServer::~AbstractServer()
{
    // TODO: Release all the sockets here
}

void AbstractServer::ChannelLogic(int i, int j, int fd, AbstractServer *thisObj)
{
    try
    {
        while (1)
        {
            if (thisObj->ChannelOperators[i][j](i, fd))
                break;
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Some ERROR!!!" << e.what() << "\n";
    }
}

int faultOccured = false;

void AbstractServer::ChannelListener(int i, AbstractServer *thisObj)
{
    int PORT = i + thisObj->PORT_BASE;
    //thisObj->SetupChannel(PORT, i);
    int new_socket;
    ssize_t valread = 0;
back:
    thisObj->smtx.lock();
    int sfd = thisObj->server_fd[i];
    struct sockaddr_in *address = thisObj->addresses[i];
    char *buff = new char[4096];

    std::cout << "\n\nServer Started at port " << PORT << " Successfully...";
    if (listen(sfd, 5) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    std::cout << "\nwaiting for incoming Connections...";
    thisObj->smtx.unlock();
    while (1)
    {
        try
        {
            if(faultOccured)
            {
                faultOccured = false;
                thisObj->ResumeHandler();
            }
            int addrlen = sizeof(struct sockaddr);
            if ((new_socket = accept(sfd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
            {
                //perror("accept");
                //exit(EXIT_FAILURE);
                printf("accept");
                delete[] buff;
                goto back;
            }
            std::cout << "\nGot an incoming request...\n";
            if (thisObj->ChannelInitializers[i](i, new_socket))
            {
                std::cout << "\nHandshake Not Success!";
                continue;
            }
            try
            {
                /*while (1)
            {
                if (thisObj->ChannelOperators[i](i, new_socket))
                    break;
            }*/
                std::vector<std::thread *> reader;
                for (int k = 0; k < thisObj->ChannelOperators[i].size(); k++)
                {
                    reader.push_back(new std::thread(&(thisObj->ChannelLogic), i, k, new_socket, thisObj));
                }
                for (int k = 0; k < thisObj->ChannelOperators[i].size(); k++)
                {
                    reader[k]->join();
                }
                throw std::runtime_error("Broken Pipe");
            }
            catch (std::exception &e)
            {
                std::cout << "Broken Pipe, Waiting for incoming Connections...";
                thisObj->ExceptionHandler();
                faultOccured = true;
            }
        }
        catch (std::exception &e)
        {
            std::cout << "Some Serious ERROR!!!" << e.what() << "\n";
            thisObj->ExceptionHandler();
            faultOccured = true;
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
