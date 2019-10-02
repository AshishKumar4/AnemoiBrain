#include "iostream"
#include "vector"
#include "list"
#include "thread"
#include "stdio.h"
#include "string"
#include "string.h"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio/read_until.hpp>

#include <chrono>

typedef struct vector3D_s
{
	float x;
	float y;
	float z;
} vector3D_t;

#pragma pack(1)
typedef struct imuDataPack_s
{
	// char 			signature[4];	// 0-4
	// quaternion_t 	orient;			// 4-20
	vector3D_t eulerOrient; // 20-32
	// vector3D_t 		linAcc;			// 32-44
	// uint32_t 		padding1;		// 44-48
	// vector3D_t 		velocity; 		// 48-60	// Absolute Velocity
	// uint16_t 		checksum;		// 60-62
	// uint8_t 		delimiter1;		// 62-63
	uint8_t delimiter2; // 63-64
} imuDataPack_t;

void syncPort(boost::asio::serial_port &mport, int timeout = 0)
{
	char c = '\0';
	while (c != '\n')
	{
		boost::asio::read(mport, boost::asio::buffer(&c, 1));
	}
}

int serialImuRead(boost::asio::serial_port &mport, imuDataPack_t &imu)
{
	boost::asio::read(mport, boost::asio::buffer(&imu, sizeof(imuDataPack_t)));
	if (imu.delimiter2 != '\n')
	{
		syncPort(mport);
		boost::asio::read(mport, boost::asio::buffer(&imu, sizeof(imuDataPack_t)));
	}
	return 0;
}

int main(int argc, const char *argv[])
{
	const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
	const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;
	imuDataPack_t imu;
	// boost::asio::basic_streambuf<std::allocator<uint8_t>> buffer;
	boost::asio::io_service m_io;
	boost::asio::serial_port mport(m_io, device);
	boost::asio::serial_port_base::baud_rate baud_rate1(baudrate);
	mport.set_option(baud_rate1);
	printf("\nReading some initial garbage data...");
	printf("\n[%d]\n\n", sizeof(imuDataPack_t));

	syncPort(mport);

	auto s = std::chrono::high_resolution_clock::now();
	while (1)
	{
		s = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < 1000; i++)
		{
			serialImuRead(mport, imu);
		}
		printf("\n[%f %f %f, %x]", imu.eulerOrient.x, imu.eulerOrient.y, imu.eulerOrient.z, imu.delimiter2);
		const auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - s).count();
		printf("\n%f Hz", (1000.0 * 1000.0) / (float)(total_time));
	}

	return 0;
}