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
#include <poll.h>
#include <future>
#include <atomic>

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

std::function<void(int)> CHANNEL_HANDLER_TABLES[] = {&ControllerInterface::_setThrottle, &ControllerInterface::_setPitch, &ControllerInterface::_setRoll, &ControllerInterface::_setYaw, &ControllerInterface::setAux1, &ControllerInterface::setAux2, &ControllerInterface::setAux3, &ControllerInterface::setAux4};
std::string CHANNEL_NAME_TABLES[] = {"throttle", "pitch", "roll", "yaw", "aux1", "aux2", "aux3", "aux4"};

char **ControlChannelBuffer = (char **)malloc(sizeof(char *) * 12);
std::string *OldControlData = new std::string[12];

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
	catch (const std::future_error &e)
	{
		std::cout << "HandShake-->Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
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

std::atomic<bool> exceptionOccured; //

int ControlBeaconMonitor(int i, int fd)
{
	try
	{
		struct pollfd ffd;
		int ret;

		ffd.fd = fd; // your socket handler
		ffd.events = POLLIN;
		ret = poll(&ffd, 1, 100); // 100ms for timeout
		switch (ret)
		{
		case -1:
			// Error
			throw "Error!";
			break;
		case 0:
			// Timeout
			if (!exceptionOccured)
			{
				printf("\nTimeout!!! Reciever data not recieved in desired time... Falling back to default procedure!");
				exceptionOccured = true;
				ControlExceptionHandler();
			}
			break;
		default:
			if (exceptionOccured)
			{
				exceptionOccured = false;
				ControlResumeHandler();
			}
			memset(ControlChannelBuffer[i], 0, 4096);
			if(recv(fd, ControlChannelBuffer[i], 4096, 0) == -1) // get your data
				return 1;
			if (strcmp(ControlChannelBuffer[i], "still alive"))
				return 1;
			//printf("\n%s", ControlChannelBuffer[i]);
			break;
		}
	}
	catch (const std::future_error &e)
	{
		std::cout << "Inner2<->Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "ERROR in ControlBeaconMonitor!!!!";
		return 1;
	}
	catch (...)
	{
		std::cout << "Some Other error in ControlBeaconMonitor!";
	}
	return 0;
}

const char *beaconSendMsg = "Yes\0";

int ControlBeaconSender(int i, int fd)
{
	fflush(stdout);
	if (exceptionOccured)
	{
		printf("\n<<ASDASDASDASDADAS>");
		fflush(stdout);
		return 1;
	}
	try
	{
		if (send(fd, beaconSendMsg, strlen(beaconSendMsg), 0) == -1)
		{
			printf("\nERROR IN BEACON SEND!");
			fflush(stdout);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		return 0;
	}
	catch (...)
	{
		printf("\nBeacon Connection broke due to some error");
		fflush(stdout);
	}
	//exceptionOccured = true;
	//ControlExceptionHandler();
	return 1;
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
			ControllerInterface::setRC_Buffered(i, uint8_t(numVal));
			CHANNEL_HANDLER_TABLES[i](numVal);
#endif
		}
#elif defined(STREAM_PROTOCOL_2)
		for (int j = 0; j < valread; j++)
		{
			CHANNEL_HANDLER_TABLES[i](int(ControlChannelBuffer[i][j]));
			ControllerInterface::setRC_Buffered(i, uint8_t(ControlChannelBuffer[i][j]));
		}
#elif defined(STREAM_PROTOCOL_3)
		// Format --> .xy.xy.xy.xy.xy.xy.xy.
		// Convert to string ->
		ControlChannelBuffer[i][valread] = '\0'; // Null terminate it
		std::string parsed, cmdNew(ControlChannelBuffer[i]);
		cmdNew = OldControlData[i] + cmdNew; // OldControlData stores previously unused data
		//printf("\n%s -- [%s]", cmdNew.c_str(), OldControlData[i].c_str());

		// Split it by '.'
		std::stringstream input_stringstream(cmdNew);
		int used = 0;
		while (std::getline(input_stringstream, parsed, '.'))
		{
			if (!parsed.length())
				continue;
			++used;
			uint8_t *tmp = (uint8_t *)parsed.c_str();
			//printf("[%d => %d] ", tmp[1], tmp[2]);
			if (parsed.length() == 2)
			{
				CHANNEL_HANDLER_TABLES[int(tmp[0] - 1)](int(tmp[1]));
				ControllerInterface::setRC_Buffered(int(tmp[0] - 1), uint8_t(int(tmp[1])));
			}
			else
			{
				//printf("\nIncorrect packet length -> %s, %d", parsed.c_str(), parsed.length());
			}
		}
		used *= 3;									  // This would give number of bytes used up
		char *chptr = ControlChannelBuffer[i] + used; // Get to that point until where it was used

		OldControlData[i] = std::string(chptr); // For the next time

#endif
	}
	catch (const std::future_error &e)
	{
		std::cout << "Inner3<->Caught a future_error with code \"" << e.code()
				  << "\"\nMessage: \"" << e.what() << "\"\n";
	}
	catch (std::exception &e)
	{
		std::cout << "Some ERROR in ControlListeners!!!!" << e.what() << "\n";
		return 1;
	}
	catch (...)
	{
		std::cout << "Some Other error in ControlListeners!";
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

int ControlServer_init(int argc, const char *argv[])
{
	try
	{
		Controls::exceptionOccured = false;
		int portBase = (argc > 3) ? stoi(std::string(argv[3])) : 8400;
		ControlServer = new Onboard::AbstractServer(portBase);
#if defined(STREAM_PROTOCOL_3)
		Onboard::Controls::OldControlData[0] = "";
		Onboard::Controls::ControlChannelBuffer[0] = new char[4096];
		Onboard::Controls::ControlChannelBuffer[1] = new char[4096];
		Onboard::Controls::ControlChannelBuffer[2] = new char[4096];
		ControlServer->AddChannels(0, &Onboard::Controls::ControlListeners, &Onboard::Controls::ControlHandshake);
		ControlServer->AddChannels(1, &Onboard::Controls::ControlBeaconMonitor, &Onboard::Controls::ControlHandshake);
		// ControlServer->AddChannels(2, Onboard::Controls::ControlBeaconSender, Onboard::Controls::ControlHandshake);
#else
		/*for (int i = 0; i < 8; i++)
        {
            Onboard::Controls::ControlChannelBuffer[i] = new char[4096];
            ControlServer.AddChannels(i, Onboard::Controls::ControlListeners, Onboard::Controls::ControlHandshake);
        }
        Onboard::Controls::ControlChannelBuffer[8] = new char[4096];
        ControlServer.AddChannels(8, Onboard::Controls::RemoteAPI_Listener, Onboard::Controls::ControlHandshake);   //*/
#endif
		ControlServer->ExceptionHandler = Onboard::Controls::ControlExceptionHandler;
		ControlServer->ResumeHandler = Onboard::Controls::ControlResumeHandler;
		ControlServer->LaunchThreads(); //*/
	}
	catch (std::exception &e)
	{
		printf("\nError in Control Server Initialisation!");
		fflush(stdin);
	}

	return 0;
}

} // namespace Onboard
