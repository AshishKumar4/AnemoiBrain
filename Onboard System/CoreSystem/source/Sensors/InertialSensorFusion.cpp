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

#include "RTIMULib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"

//  global variables

namespace InertialSensorFusion
{

static RTIMUSettings *settings;
static RTPressure *pressure;
static RTIMU_DATA imuData;
static RTIMU *imu;
static RTIMUMagCal *magCal;
static RTIMUAccelCal *accelCal;

static bool magMinMaxDone;
static bool accelEnables[3];
static int accelCurrentAxis;

namespace
{
quaternion_t 	orientation_estimate;
vector3D_t 		orientationEuler_estimate;
vector3D_t 		acceleration_estimate;
float		 	altitude_estimate;

std::thread* 	inertialFusionThread;
} // namespace

int sampleCount = 0;
int sampleRate = 0;
uint64_t rateTimer;
uint64_t displayTimer;
uint64_t now;

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
	char *settingsFile = (argc > 2) ? argv[2] : (char *)"RTIMULib";

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
	imu->setCompassEnable(true);

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