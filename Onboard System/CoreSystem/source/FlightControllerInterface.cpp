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

#include "ControllerInterface.hpp"
#include "FlightControllerInterface.hpp"

#include "common.hpp"

#include "Sensors/Sensors.hpp"
#include "Sensors/InertialMeasurement.hpp"
#include "Sensors/Location.hpp"

// #include "specificDefs.h"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For SPI Based Communication------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(ONBOARD_SPI_PROTOCOL)

#ifndef MODE_DEBUG_NO_FC
#include "LowLevel/SPIdrivers.h"
#include "wiringPiSPI.h"
#endif

int fd;

#define TIMEOUT_VAL 20

/*
        The new handshake mechanism works like this ->
        Send a special msg, then wait for an amount of time for it to arrive. 
        if it arrives, validate, else print handshake failed or timed out
    */

unsigned char checksum(unsigned char *buf, int len)
{
	unsigned char tt = 0;
	for (int i = 0; i < len - 1; i++)
	{
		tt += buf[i];
	}
	return tt;
}

int SPI_handshake()
{
	int ht = REQ2_SIGNAL;
#ifndef MODE_DEBUG_NO_FC
	SPI_ReadWrite((int)fd, (uintptr_t)&ht, (uintptr_t)&ht, (size_t)1); //wiringPiSPIDataRW(0, (unsigned char*)&ht, 1);
#endif
	//nanosleep(t100000n, NULL);
	return 0;
}

int IssueCommand()
{
	if (!SPI_handshake())
	{
		uint8_t *ht = ((uint8_t *)pp);
		uint8_t *hr = ((uint8_t *)&rff);
		pp->checksum = checksum(ht, sizeof(ControlPackets));
		// uint8_t tb = 0;
#ifndef MODE_DEBUG_NO_FC
		SPI_ReadWrite((int)fd, (uintptr_t)ht, (uintptr_t)hr, (size_t)sizeof(ControlPackets));
#endif
		std::cout << "Successfully Issued Command\n";
		return 0;
	}
	return 1;
}

void Channel_Updater()
{
	std::cout << "\nSPI Updater Initialized...";
	uint8_t *tbo = (uint8_t *)ppold;
	uint8_t *tb = (uint8_t *)pp;
	while (1)
	{
		/*
                Well, Basically, Firstly, check if the new data that we are gonna send is identical to the older, we don't need to send anything!
            */
		for (int i = 0; i < sizeof(ControlPackets); i++)
		{
			if (tb[i] != tbo[i])
				goto update;
		}
		goto skip;
	update:
		memcpy((void *)tbo, (void *)tb, sizeof(ControlPackets));
		if (IssueCommand())
		{
			std::cout << "Couldn't Issue the command\n";
		}
	skip:
		nanosleep(t10000n, NULL);
	}
}

void Raw_Init(int argc, const char *argv[])
{
#ifndef MODE_DEBUG_NO_FC
	//fd = SPI_init("/dev/spidev0.0");
	fd = wiringPiSPISetup(0, 500000);
#endif
}

void sendCommand(uint8_t val, uint8_t channel)
{
	// First send the value, then the channel, then a dummy and check the returned checksum.
	// if its good, okay otherwise repeat.
	Main_Mutex.lock();
	uint8_t *bv = new uint8_t;
	uint8_t *bc = new uint8_t;
	uint8_t *tmp1 = new uint8_t;
	uint8_t *tmp2 = new uint8_t;
	uint8_t *t1 = new uint8_t;
	uint8_t *t2 = new uint8_t;
	int counter = 0;
	int flg1 = 0, flg2 = 0;
back:
	printf("\n[Attempting send %d to %d <%d, %d>", val, channel, flg1, flg2);
	*bv = val;
	*bc = channel;

#ifndef MODE_DEBUG_NO_FC
	wiringPiSPIDataRW(0, bv, 1); // Send the value
#endif
	nanosleep(t100000n, NULL);

#ifndef MODE_DEBUG_NO_FC
	wiringPiSPIDataRW(0, bc, 1); // Send the channel, recieve the value
#endif
	nanosleep(t100000n, NULL);

	if (*bc == (val ^ 0xff) || flg1)
	{
		printf(" [correct val %d %d, %d] ", *bc, (val ^ 0xff), flg1);
		*tmp1 = val;
		*t1 = (*bc ^ 0xff);
		flg1 = 1;
	}
	else
	{
		printf(" [false val %d %d, %d] ", *bc, (val ^ 0xff), flg1);
		*tmp1 = 0xff;
		flg1 = 0;
	}

#ifndef MODE_DEBUG_NO_FC
	wiringPiSPIDataRW(0, tmp1, 1); // Send confirmation of reception, recieve channel
#endif
	nanosleep(t100000n, NULL);

	if (*tmp1 == (channel ^ 0xff) || flg2)
	{
		printf(" [correct channel %d %d, %d] ", *tmp1, (channel ^ 0xff), flg2);
		*tmp2 = channel;
		*t2 = (*tmp1 ^ 0xff);
		flg2 = 1;
	}
	else
	{
		printf(" [false channel %d %d, %d] ", *tmp1, (channel ^ 0xff), flg2);
		*tmp2 = 0xff;
		flg2 = 0;
	}

#ifndef MODE_DEBUG_NO_FC
	wiringPiSPIDataRW(0, tmp2, 1); // Send confirmation of reception, recieve something
#endif
	nanosleep(t100000n, NULL);

	/*
            If the value was wrong but channel was right, send the value only;
            and vice versa
        */
	if (flg1 && flg2)
	{
		printf("\tSuccess!, %d, %d", *t1, *t2);
		delete bv;
		delete bc;
		delete tmp1;
		delete tmp2;
		delete t1;
		delete t2;
		Main_Mutex.unlock();
		return void;
	}
	else
	{
		printf("\t[Retrying]");
		goto back;
	}
}
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For I2C Based Communication------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined I2C_PROTOCOL
#include "LowLevel/I2Cdrivers.h"

int IssueCommand()
{
}

void Raw_Init(int argc, const char *argv[])
{
}

void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ---------------------------------------------For Nrf24L01 Communication------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined NRF24L01_SPI_PROTOCOL

#define NRF24_INAV_PROTOCOL

// Uses INAV Protocol

#include "RadioDrivers.cpp"

void Sensors_Updater()
{
	return;
}

int IssueCommand()
{
	NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
}

void Channel_Updater()
{
	while (1)
	{
	}
}

void Raw_Init(int argc, const char *argv[])
{
}

void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------------For MavLink based Communication---------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_MAVLINK

int IssueCommand()
{
}

void Raw_Init(int argc, const char *argv[])
{
}

void sendCommand(uint8_t val, uint8_t channel)
{
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------For MSP Based Serial directly to FC-------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MSP_Serial_PROTOCOL

#include <msp/msp_msg.hpp>
#include <msp/FlightController.hpp>
#include <msp/FirmwareVariants.hpp>

#define TIMESLICE 0.01

fcu::FlightController *FlController;

#if defined(MSP_DUAL_COM)
fcu::FlightController *FlControllerRC;
#endif

std::function<void()> channelUpdaterFunc;

msp::FirmwareVariant FCvariant;
std::atomic_bool exitFlag;

int IssueCommand()
{
	return 0;
}

uint16_t rcExpand(uint8_t val) // Basically map val from 0, 255 to 1000 to 2000
{
	uint16_t aa = 1000 + uint16_t(int(3.938 * double(val))); // The formula to map ranges
	if (aa > 2000)
		aa = 2000;

	return aa;
}

void Sensors_Updater()
{
	return;
}

void dependent_Channel_Updater();

#if defined(MSP_DUAL_COM)
void independent_Channel_Updater()
{
	try
	{
#if !defined(UNIFIED_UPDATER)
		while (1)
#endif
		{
			if (exitFlag)
				return;
			FlControllerRC->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), rcExpand(RC_DATA[AUX3]), rcExpand(RC_DATA[AUX4]));
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
		}
	}
	catch (const std::exception &e)
	{
		printf("\nFalling back to primary MSP port...");
		fflush(stdout);
		FlControllerRC = FlController;
		channelUpdaterFunc = &dependent_Channel_Updater;
	}
}
#endif

void dependent_Channel_Updater()
{
#if !defined(UNIFIED_UPDATER)
	while (1)
#endif
	{
		if (exitFlag)
			return;
		Main_Mutex.lock();
		FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), rcExpand(RC_DATA[AUX3]), rcExpand(RC_DATA[AUX4]));
		Main_Mutex.unlock();
#if !defined(UNIFIED_UPDATER)
		std::this_thread::sleep_for(std::chrono::microseconds(int(TIMESLICE * 1000 * 1000)));
#endif
	}
}

void Channel_Updater()
{
	try
	{
		channelUpdaterFunc();
	}
	catch (std::exception &e)
	{
		std::cout << "Error in Channel Updater " << e.what();
		//Main_Mutex.unlock();
	}
}

void Raw_Init(int argc, const char *argv[])
{
	const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
	const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
	exitFlag = true;

	std::cout << "\n\tAttempting to connect to the Flight Controller...\n";

	std::chrono::high_resolution_clock::time_point start, end;
	fcu::FlightController *fcdesc = new fcu::FlightController();
	// FlController.store(fcdesc);
	FlController = fcdesc;

	auto logLevel = msp::client::LoggingLevel::INFO;
	std::cout << logLevel;
	fcdesc->setLoggingLevel(logLevel);

	// wait until connection is established
	// get unique box IDs
	try
	{
		start = std::chrono::high_resolution_clock::now();
		fcdesc->connect(device, baudrate, 1, true); //initialise();
		end = std::chrono::high_resolution_clock::now();
		if (!fcdesc->isConnected())
		{
			std::cout << "FC Not Connected, exiting\n";
			exit(0);
		}
		std::cout << "FC connected, ready after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
		fflush(stdout);
	}
	catch (std::exception &e)
	{
		std::cout << "ERROR OCCURED! " << e.what();
		fflush(stdout);
	}
	// on cleanflight, we need to enable the "RX_MSP" feature
	FCvariant = fcdesc->getFwVariant();
	std::string firmwareVariant = msp::firmwareVariantToString(FCvariant);

	std::cout << "\nFC Recognized as " << firmwareVariant << " and successfully connected!\n";

	ControllerInterface::MainFC = new FlightController("master", "multiwii", firmwareVariant, (uintptr_t)fcdesc);

	std::cout << "Armed? " << fcdesc->isArmed() << std::endl;

	// disarm the FC
	std::cout << "Disarming..." << std::endl;
	start = std::chrono::high_resolution_clock::now();
	fcdesc->disarm_block();
	end = std::chrono::high_resolution_clock::now();

	if (!fcdesc->isArmed())
	{
		std::cout << "disarmed after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
	}

	channelUpdaterFunc = &dependent_Channel_Updater;
#if defined(MSP_DUAL_COM)
	try
	{
		const std::string device2 = (argc > 4) ? std::string(argv[4]) : "/dev/ttyUSB0";
		const size_t baudrate2 = (argc > 5) ? std::stoul(argv[5]) : 115200;
		FlControllerRC = new fcu::FlightController();
		if (device2 == device)
		{
			throw std::runtime_error("Same devices");
		}
		else if (!FlControllerRC->connect(device2, baudrate2, 1))
		{
			throw std::runtime_error("Could not connect to device");
		}
		else
		{
			printf("\nRC Controller Independent of Main Controller Established! ");
			channelUpdaterFunc = &independent_Channel_Updater;
		}
	}
	catch (std::exception &e)
	{
		printf("\nProblem connecting to RC serial port, Falling back to default...");
		std::cout << e.what();
		FlControllerRC = FlController; // Failed to bind to the other device
	}
#endif
}

void sendCommand(uint8_t val, uint8_t channel)
{
	Main_Mutex.lock();
	FlController->setRc(rcExpand(RC_DATA[ROLL]), rcExpand(RC_DATA[PITCH]), rcExpand(RC_DATA[YAW]), rcExpand(RC_DATA[THROTTLE]), rcExpand(RC_DATA[AUX1]), rcExpand(RC_DATA[AUX2]), rcExpand(RC_DATA[AUX3]), rcExpand(RC_DATA[AUX4]));
	Main_Mutex.unlock();
}

void destroyFlightControllerObjs()
{
	exitFlag = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	if (FlController != FlControllerRC)
	{
		FlControllerRC->disconnect();
		delete FlControllerRC;
	}
	FlController->disconnect();
	delete FlController;
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -------------------------------------For AirSim Simple FC, through provided APIs---------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined MODE_AIRSIM

#if defined(AIRSIM_MODE_SOCKET)

#elif defined(AIRSIM_MODE_API)
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "rpc/server.h"

std::atomic<msr::airlib::MultirotorRpcLibClient *> client;

#define TIMESLICE 0.001

/*

def valMap(i, imin, imax, omin, omax):
    aa = omin + (((omax - omin) / (imax - imin)) * (i - imin))  # The formula to map ranges
    return aa
*/

inline float rcShrink(uint8_t val, float omin = -1.0, float omax = 1.0)
{
	float aa = omin + (((omax - omin) / (255.0)) * int(val));
	return aa;
}

int IssueCommand(int threadId)
{
	//NRF24_Send((uintptr_t)pp, (uintptr_t)&rff, sizeof(ControlPackets));
	return 0;
}

quaternion_t AIRSIM_oritentation;
GeoPoint_t AIRSIM_location;
float AIRSIM_velocity;
vector3D_t AIRSIM_velocityAbs;
vector3D_t AIRSIM_velocityRel;
vector3D_t AIRSIM_euleroritentation;

bool exitFlag = false;

/*

Airsim Coordinate system is fucked up

North is X axis, East is Y axis, Clockwise from north is +ve angle and viceversa

We do a few cartesian transforms to make North as Y axis, East as X axis, Counter-Clockwise as +ve

 */

void Sensors_Updater()
{
	float theta, tmpx, tmpy;
	while (1)
	{
		try
		{
			if (exitFlag)
				return;
#if !defined(MODE_DEBUG_NO_FC)
			Main_Mutex.lock();
			auto state = client.load()->getMultirotorState();
			Main_Mutex.unlock();
			auto orien = state.getOrientation();
			AIRSIM_oritentation.set(orien.w(), orien.x(), orien.y(), orien.z());

			auto velocity = state.kinematics_estimated.twist.linear;
			AIRSIM_velocityAbs.set(velocity.y(), velocity.x(), velocity.z());

			theta = AIRSIM_euleroritentation.z;
			tmpy = AIRSIM_velocityAbs.y * cos(theta) + AIRSIM_velocityAbs.x * sin(theta);
			tmpx = -AIRSIM_velocityAbs.y * sin(theta) + AIRSIM_velocityAbs.x * cos(theta);
			AIRSIM_velocityRel.set(tmpx, tmpy, velocity.z());

			auto location = state.getPosition();
			AIRSIM_location.set(location.y(), location.x(), -location.z());

			auto euler = eulerFromQuaternion(AIRSIM_oritentation);
			AIRSIM_euleroritentation.set(euler.x, euler.y, euler.z);
			//Main_Mutex.unlock();
			//std::this_thread::sleep_for(std::chrono::miliseconds(1));
#endif
		}
		catch (const std::future_error &e)
		{
			std::cout << "<Sensor_Updater>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			//Main_Mutex.unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in CLI Monitor " << e.what();
			//Main_Mutex.unlock();
		}
	}
}

void Channel_Updater()
{

#if !defined(UNIFIED_UPDATER)
	std::thread sensors(Sensors_Updater);
	while (1)
#endif
	{
		try
		{
			if (exitFlag)
				return;
#if !defined(MODE_DEBUG_NO_FC)
			Main_Mutex.lock();
			client.load()->moveByAngleThrottleAsync(rcShrink(255 - RC_DATA[PITCH]), rcShrink(RC_DATA[ROLL]), rcShrink(RC_DATA[THROTTLE], 0, 5), rcShrink(RC_DATA[YAW], -10.0, 10.0), TIMESLICE);
			Main_Mutex.unlock();
#endif
#if !defined(UNIFIED_UPDATER)
			std::this_thread::sleep_for(std::chrono::microseconds(int(TIMESLICE * 1000.0 * 1000.0)));
#endif
		}
		catch (const std::future_error &e)
		{
			std::cout << "<Channel_Updater>Caught a future_error with code \"" << e.code()
					  << "\"\nMessage: \"" << e.what() << "\"\n";
			//Main_Mutex.unlock();
		}
		catch (std::exception &e)
		{
			std::cout << "Error in CLI Monitor " << e.what();
			//Main_Mutex.unlock();
		}
	}
#if !defined(UNIFIED_UPDATER)
	sensors.join();
#endif
}

void Raw_Init(int argc, const char *argv[])
{
	const std::string ip = (argc > 1) ? std::string(argv[1]) : "localhost";
	const uint16_t port = (argc > 2) ? std::stoi(argv[2]) : 41451;
	//rpc::server srv(8080);
#if !defined(MODE_DEBUG_NO_FC)
	//cout << "Press Enter to enable API control" << endl; cin.get();
	msr::airlib::MultirotorRpcLibClient *obj = new msr::airlib::MultirotorRpcLibClient(ip, port);
	client.store(obj);
	printf("\nEnabling AirSim API Control");
	obj->enableApiControl(true);

	//cout << "Press Enter to arm the drone" << endl; cin.get();
	printf("\nArming the multirotor");
	obj->armDisarm(true);

	//cout << "Press Enter to takeoff" << endl; cin.get();
	//client->takeoffAsync(5); //*/
#else
	obj = nullptr;
#endif
	printf("\nInitialization complete");
	ControllerInterface::MainFC = new FlightController("master", "AirSim", "Simple FC", (uintptr_t)obj);
}

void sendCommand(uint8_t val, uint8_t channel)
{
	//client.moveByAngleThrottleAsync(rcShrink(RC_DATA[PITCH]), rcShrink(RC_DATA[ROLL]), rcShrink(RC_DATA[THROTTLE], 0, 10), rcShrink(RC_DATA[YAW], -6, 6), 10);
}

void destroyFlightControllerObjs()
{
	exitFlag = true;
	printf("\nDestroying Objects...");
	fflush(stdout);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	delete client.load();
}

#endif
#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* --------------------------------------For Testing without FC, on development PC----------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if !defined(MODE_AIRSIM) && (defined(MODE_DEBUG_NO_FC) || defined(FAKE_PROTOCOL))

int IssueCommand()
{
	return 0;
}

void Channel_Updater()
{
	while (1)
		;
}

void Raw_Init(int argc, const char *argv[])
{
}

void sendCommand(uint8_t val, uint8_t channel)
{
	printf("\n[%d ------> %d", val, channel);
}

#endif

/* ------------------------------------------------------------------------------------------------------------------------ */
/* -----------------------------------------------RX Definitions ends here------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------Protocol Specific Data Gathering---------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------------ */

#if defined(MSP_Serial_PROTOCOL)
#endif

#if defined(MODE_AIRSIM)
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

bolatile char *bb = new char[4096];

struct MSP_Packet
{
	char *buf;
	int size;

	MSP_Packet(char *buf, int sz) : buf(buf), size(sz){};
};

MSP_Packet MSP_Agent(char *buf, int size);

volatile msp::MSP *msp_agent;

MSP_Packet MSP_Agent(char *buf, int size)
{
	try
	{
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