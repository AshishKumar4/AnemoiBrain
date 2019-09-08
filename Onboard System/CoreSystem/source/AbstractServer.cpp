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
#include <future>
#include <atomic>

#include "AbstractServer.hpp"

namespace Onboard
{

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------------Main Program------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

int opt = 1;

void AbstractServer::AddChannels(int index, func_iii_t maincode, func_iii_t initializer) // Should be called once
{
	if (ChannelOperators.size() <= index)
	{
		// std::vector<func_iii_t> vec;
		// vec.push_back(function);
		ChannelOperators.push_back(maincode); // (vec)
		ChannelInitializers.push_back(initializer);
		server_fd.push_back(0);
		addresses.push_back(NULL);
	}
	else
	{
		// if (ChannelOperators[index].size() <= subchannel)
		// {
		// 	ChannelOperators[index].push_back(function);
		// }
		// else
		// {
		// 	ChannelOperators[index][subchannel] = function;
		// }
		ChannelInitializers[index] = initializer;
		ChannelOperators[index] = maincode;
	}
}

void AbstractServer::CreateChannels(int index, func_iii_t maincode, func_iii_t initializer, bool initPort) // Should be called once
{
	AddChannels(index, maincode, initializer);
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

void AbstractServer::ChannelLogic(int i, int fd, AbstractServer *thisObj)
{
	while (1)
	{
		try
		{
			if (thisObj->ChannelOperators[i](i, fd))
			{
				printf("\nGOT Connection Break [%d]!!! EXITING....", i);
				return;
			}
		}
		catch (std::exception &e)
		{
			std::cout << "Error in Channel Logic loop!" << e.what();
		}
	}
}

namespace
{

int faultOccured = false;
std::mutex FaultTrigger;

} // Anonymous namespace

void AbstractServer::triggerFault()
{
	FaultTrigger.lock();
	if (!faultOccured)
		this->ExceptionHandler();
	faultOccured = true;
	FaultTrigger.unlock();
}

void AbstractServer::manageFault()
{
	FaultTrigger.lock(); // Only let a Single thread trigger the Fault!
	if (faultOccured)
		this->ResumeHandler();
	faultOccured = false;
	FaultTrigger.unlock();
}

void AbstractServer::ChannelListener(int i, AbstractServer *thisObj)
{
	int addrlen = sizeof(struct sockaddr);
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
	fflush(stdout);
	if (listen(sfd, 5) < 0)
	{
		perror("listen");
	}
	std::cout << "\nwaiting for incoming Connections...";
	fflush(stdout);
	thisObj->smtx.unlock();

	try
	{
		while (1)
		{
			try
			{
				// Listen for incoming connection...
				if ((new_socket = accept(sfd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
				{
					printf("accept");
					goto back;
				}
				std::cout << "\nGot an incoming connection request...\n";
				fflush(stdout);
				if (thisObj->ChannelInitializers[i](i, new_socket))
				{
					perror("\nHandshake Not Success!");
					continue;
				}
				// Resume any Fault Handlers previously called
				if (faultOccured)
				{
					thisObj->manageFault();
				}
				try
				{
					// Launch actual worker threads
					// std::vector<std::thread *> reader;
					// for (int k = 0; k < thisObj->ChannelOperators[i].size(); k++)
					// {
					// 	reader.push_back(new std::thread(&(thisObj->ChannelLogic), i, k, new_socket, thisObj));
					// }
					// for (int k = 0; k < reader.size(); k++)
					// {
					// 	reader[k]->join();
					// }
					thisObj->ChannelLogic(i, new_socket, thisObj);
					throw std::runtime_error("Broken Pipe");
				}
				catch (std::exception &e)
				{
					perror(e.what());
					thisObj->triggerFault();
				}
			}
			catch (std::exception &e)
			{
				perror(e.what());
				thisObj->triggerFault();
			}
		}
	}
	catch (const std::future_error &e)
	{
		std::cout << "<AbstractServer::ChannelListener>[03]Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
		fflush(stdout);
	}
	catch (const std::exception &e)
	{
		std::cout << "Error in AbstractServer::ChannelListener --> " << e.what() << '\n';
		fflush(stdout);
	}
	perror("\nReached the end of Channel Listener, Shouldn't have been here!");
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
		// exit(EXIT_FAILURE);
	}

	// Forcefully attaching socket to the port
	if (setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
	{
		perror("setsockopt");
		//exit(EXIT_FAILURE);
	}
	address->sin_family = AF_INET;
	address->sin_addr.s_addr = INADDR_ANY;
	address->sin_port = htons(port);

	// Forcefully attaching socket to the given port
	//bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in));
	if (::bind(sfd, (struct sockaddr *)address, sizeof(struct sockaddr_in)) < 0)
	{
		perror("bind failed");
		// exit(EXIT_FAILURE);
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
		// exit(EXIT_FAILURE);
	}
	std::cout << "Socket for port " << port << " Created Successfully!" << std::endl;
	smtx.unlock();
	return 0;
}
} // namespace Onboard
