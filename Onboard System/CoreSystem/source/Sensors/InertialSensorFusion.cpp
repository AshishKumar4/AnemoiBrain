#include <string>
#include <iostream>
#include <vector>
#include <iterator>
#include <sstream>
#include <thread> // std::thread
#include <mutex>
#include <stdexcept>
#include <future>

#include "common.hpp"

#include "Sensors/Sensors.hpp"
#include "Sensors/drivers/GeoEstimator.hpp"
#include "Sensors/drivers/InertialEstimator.hpp"

/***********************************************************************************************/
/*************************** Systems to generate Sensor Fusion Data ****************************/
/***********************************************************************************************/

#if defined(MODE_REALDRONE)

#include "chrono"

#if defined(USE_EXTERNAL_SENSORBOARD)

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio/read_until.hpp>

#else

#include "RTIMULib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"

#endif 
//  global variables

namespace InertialSensorFusion
{

static bool magMinMaxDone;
static bool accelEnables[3];
static int accelCurrentAxis;

namespace
{
quaternion_t orientation_estimate;
vector3D_t orientationEuler_estimate;
vector3D_t acceleration_estimate;
float altitude_estimate;

std::thread *inertialFusionThread;
} // namespace

int sampleCount = 0;
int sampleRate = 0;
uint64_t rateTimer;
uint64_t displayTimer;
uint64_t now;

#if defined(USE_EXTERNAL_SENSORBOARD)

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

static imuDataPack_t imu;
static boost::asio::serial_port* mport;
static boost::asio::io_service m_io;

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

void InertialSensor_Fusion_worker()
{
	auto s = std::chrono::high_resolution_clock::now();
	while (1)
	{
		s = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < 1000; i++)
		{
			serialImuRead(*mport, imu);
			orientationEuler_estimate.set(imu.eulerOrient.x, imu.eulerOrient.y, imu.eulerOrient.z);
		}
		printf("\n[%f %f %f, %x]", imu.eulerOrient.x, imu.eulerOrient.y, imu.eulerOrient.z, imu.delimiter2);
		const auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - s).count();
		printf("\n%f Hz", (1000.0 * 1000.0) / (float)(total_time));
	}
}

int InertialSensor_Fusion_init(int argc, char **argv)
{
	char *device = (argc > 6) ? argv[6] : (char *)"/dev/ttyUSB0";
	int baudrate = (argc > 7) ? atoi(argv[7]) : 115200;

	mport = new boost::asio::serial_port(m_io, device);
	boost::asio::serial_port_base::baud_rate baud_rate1(baudrate);
	mport->set_option(baud_rate1);

	printf("\nReading some initial garbage data...");
	printf("\n[%d]\n\n", sizeof(imuDataPack_t));

	syncPort(*mport);

	inertialFusionThread = new std::thread(InertialSensor_Fusion_worker);
	return 0;
}

#else


static RTIMUSettings *settings;
static RTPressure *pressure;
static RTIMU_DATA imuData;
static RTIMU *imu;
static RTIMUMagCal *magCal;
static RTIMUAccelCal *accelCal;

void InertialSensor_Fusion_worker()
{
	while (1)
	{
		usleep(imu->IMUGetPollInterval() * 1000);
		// auto
		while (imu->IMURead())
		{
			RTIMU_DATA imuData = imu->getIMUData();

			//  add the pressure data to the structure

			if (pressure != NULL)
			{
				pressure->pressureRead(imuData);
				altitude_estimate = RTMath::convertPressureToHeight(imuData.pressure);
			}

			const auto pose = imuData.fusionPose;
			orientationEuler_estimate.set(pose.x(), pose.y(), pose.z());

			const auto qpose = imuData.fusionQPose;
			orientation_estimate.set(qpose.scalar(), qpose.x(), qpose.y(), qpose.z());

			const auto accel = imuData.accel;
			acceleration_estimate.set(accel.x(), accel.y(), accel.z());

			sampleCount++;

			now = RTMath::currentUSecsSinceEpoch();

			//  display 10 times per second

			if ((now - displayTimer) > 100000)
			{
				printf("Sample rate %d: %s\n", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));

				if (pressure != NULL)
				{
					printf("Pressure: %4.1f, height above sea level: %4.1f, temperature: %4.1f\n",
						   imuData.pressure, RTMath::convertPressureToHeight(imuData.pressure), imuData.temperature);
				}

				fflush(stdout);
				displayTimer = now;
			}

			//  update rate every second

			if ((now - rateTimer) > 1000000)
			{
				sampleRate = sampleCount;
				sampleCount = 0;
				rateTimer = now;
			}
		}
	}
}

int InertialSensor_Fusion_init(int argc, char **argv)
{
	char *settingsFile = (argc > 6) ? argv[6] : (char *)"RTIMULib";

	printf("RTIMULibCal - using %s.ini\n", settingsFile);

	settings = new RTIMUSettings(settingsFile);

	bool mustExit = false;
	imu = NULL;

	if (imu != NULL)
		delete imu;

	imu = RTIMU::createIMU(settings);
	pressure = RTPressure::createPressure(settings);

	if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
	{
		printf("No IMU found\n");
		exit(1);
	}

	//  set up IMU

	imu->IMUInit();

	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(false);

	if (pressure != NULL)
	{
		pressure->pressureInit();
		std::cout << "\nCompatible Barometer detected, initializing...";
	}
	else
	{
		std::cout << "\nBarometer not detected!";
	}
	//  set up for calibration run

	imu->setCompassCalibrationMode(true);
	imu->setAccelCalibrationMode(true);
	magCal = new RTIMUMagCal(settings);
	magCal->magCalInit();
	magMinMaxDone = false;
	accelCal = new RTIMUAccelCal(settings);
	accelCal->accelCalInit();

	rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();
	inertialFusionThread = new std::thread(InertialSensor_Fusion_worker);
	return 0;
}

#endif

quaternion_t getOrientation()
{
	return quaternion_t();
}

vector3D_t getOrientationEuler()
{
	return vector3D_t(0, 0, 0);
}

vector3D_t getAcceleration()
{
	return vector3D_t(0, 0, 0);
}

float getAltitude() // This system is 10/11 DOF compatible
{
	return 0;
}

} // namespace InertialSensorFusion

#endif
