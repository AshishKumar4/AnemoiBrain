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
#include "Sensors/Location.hpp"
#include "Sensors/InertialMeasurement.hpp"

/***********************************************************************************************/
/*************************** Systems to generate Sensor Fusion Data ****************************/
/***********************************************************************************************/

#if defined(MODE_REALDRONE)

#include "RTIMULib.h"
#include "RTIMUMagCal.h"
#include "RTIMUAccelCal.h"

//  global variables

static RTIMUSettings 	*settings;
static RTIMU_DATA 		imuData;
static RTIMU 			*imu;
static RTIMUMagCal 		*magCal;
static RTIMUAccelCal 	*accelCal;

static bool 		magMinMaxDone;
static bool 		accelEnables[3];
static int 			accelCurrentAxis;

quaternion_t 		Real_IMU_oritentation;
vector3D_t 			Real_IMU_euleroritentation;

GeoPoint_t 			REAL_location;
vector3D_t 			REAL_velocity;

namespace {
	
}

void Sensor_Fusion_worker()
{
	while(1)
	{

	}
}

int Sensor_Fusion_init(int argc, char **argv)
{
    char *settingsFile = (argc > 2) ? argv[2]: (char*)"RTIMULib";

    printf("RTIMULibCal - using %s.ini\n", settingsFile);

    settings = new RTIMUSettings(settingsFile);

    bool mustExit = false;
    imu = NULL;
	
    if (imu != NULL)
        delete imu;

    imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  set up IMU

    imu->IMUInit();

    //  set up for calibration run

    imu->setCompassCalibrationMode(true);
    imu->setAccelCalibrationMode(true);
    magCal = new RTIMUMagCal(settings);
    magCal->magCalInit();
    magMinMaxDone = false;
    accelCal = new RTIMUAccelCal(settings);
    accelCal->accelCalInit();
	return 0;
}

#endif