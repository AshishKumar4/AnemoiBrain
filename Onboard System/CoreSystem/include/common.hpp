#pragma once

#include "configurations.h"
#include "iostream"
#include "vector"
#include <cmath>
#include <thread>
#include <rpc/msgpack.hpp>

using namespace std;

class vector3D_t
{
public:
    float x;
    float y;
    float z;
	MSGPACK_DEFINE_MAP(this->x, this->y, this->z);

    vector3D_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    vector3D_t(float xval, float yval, float zval) : x(xval), y(yval), z(zval)
    {
    }
	
    explicit vector3D_t(uint8_t arr[]) : x((float)arr[0]), y((float)arr[1]), z((float)arr[2])
    {
    }

    void set(float xval, float yval, float zval)
    {
        x = xval;
        y = yval; 
        z = zval;
    }

    void set(vector3D_t vec)
    {
        x = vec.x;
        y = vec.y; 
        z = vec.z;
    }
};

class quaternion_t 
{
public: 

    float _w;
    float _x;
    float _y;
    float _z;
	MSGPACK_DEFINE_MAP(this->_w, this->_x, this->_y, this->_z);

    quaternion_t() : _w(1), _x(0), _y(0), _z(0)
    {
    }

    quaternion_t(float w, float x, float y, float z) : _w(w), _x(x), _y(y), _z(z)
    {
    }

    void set(float w, float x, float y, float z)
    {
        this->_w = w;
        this->_x = x;
        this->_y = y;
        this->_z = z;
    }

    float w()
    {
        return _w;
    }

    float x()
    {
        return _x;
    }

    float y()
    {
        return _y;
    }

    float z()
    {
        return _z;
    }
};

class GeoPoint_t
{
public: 
    float x;
    float y;
    float z;
	MSGPACK_DEFINE_MAP(this->x, this->y, this->z);

    GeoPoint_t() : x(0), y(0), z(0)
    {
    }
    
    GeoPoint_t(float xval, float yval, float zval) : x(xval), y(yval), z(zval)
    {
    }

    void set(float xval, float yval, float zval)
    {
        x = xval;
        y = yval; 
        z = zval;
    }

    void set(const GeoPoint_t &point)
    {
        x = point.x;
        y = point.y;
        z = point.z;
    }
};

class data_imu_t
{
public: 
	quaternion_t orien;

	// MSGPACK_DEFINE_MAP(this->acc, this->gyro, this->mag);
	MSGPACK_DEFINE_MAP(this->orien);

	explicit data_imu_t(const quaternion_t &orien) : orien(orien)
	{
	}

	// data_imu_t(vector3D_t &acc, vector3D_t &gyro, vector3D_t &mag) : acc(acc), gyro(gyro), mag(mag)
	// {
	// }

	// data_imu_t(vector3D_t acc, vector3D_t gyro, vector3D_t mag) : acc(acc), gyro(gyro), mag(mag)
	// {
	// }
};

class image_t 
{
public:
	vector<uint8_t> buff;
	MSGPACK_DEFINE_MAP(buff);
};

class DroneState_t
{
public:
	data_imu_t  imu;
	vector3D_t 	vel;
	float 		altitude;
	float 		heading;	// in Radians
	MSGPACK_DEFINE_MAP(this->imu, this->vel, this->altitude, this->heading);

	DroneState_t(const data_imu_t &imu, const vector3D_t &vel, float altitude, float heading) : imu(imu), vel(vel), altitude(altitude), heading(heading)
	{
	}

	// DroneState_t(data_imu_t imu, vector3D_t vel, float altitude, float heading) : imu(imu), vel(vel), altitude(altitude), heading(heading)
	// {
	// }
};

class SpinLock 
{
    std::atomic_flag locked;
public:
    void lock() 
	{
        while (locked.test_and_set(std::memory_order_acquire)) 
		{ 
        	//std::this_thread::yield(); //<- this is not in the source but might improve performance. 
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    void unlock() 
	{
        locked.clear(std::memory_order_release);
    }
};

/*
    This is our convention, counter clockwise, 0 to 360 in every direction
*/

inline float clamp(float val, float imin, float imax, float omin, float omax)
{
    float aa = omin + (((omax - omin) / (imax - imin)) * (val - imin));
    return aa;
}

#define PI                  3.14159265358979323846
#define CONST_PIby180       PI/180
#define CONST_180byPI       180/PI

inline float degreesToRads(float degree)
{
    return degree * (CONST_PIby180); //(degree/180)*PI;
}

inline float radsToDegrees(float degree)
{
    return degree * CONST_180byPI;
}

inline float getConventionalDegrees(float rads)
{
    // 0 -> North
    // 90 -> east
    // 180 -> south
    // 270 ->west
    float h = clamp(rads, -PI, PI, 180, -180);
    if (h < 0)
    {
        h = 360 + h;
    }
    return h;
}

inline float circularToSignAngle(float angle)
{
    if(angle > 180)
        return angle - 360;
    return angle;
}

inline vector3D_t eulerFromQuaternion(const quaternion_t &orien)
{
    try
    {
        vector3D_t oo;
        /*
        Quaternion to Euler
    */
        //std::cout << ">>>" << orien << "<<<" << std::endl;
        double heading, roll, pitch, yaw;
        double ysqr = orien._y * orien._y;

        // roll (x-axis rotation)
        double t0 = +2.0f * (orien._w * orien._x + orien._y * orien._z);
        double t1 = +1.0f - 2.0f * (orien._x * orien._x + ysqr);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0f * (orien._w * orien._y - orien._z * orien._x);
        t2 = ((t2 > 1.0f) ? 1.0f : t2);
        t2 = ((t2 < -1.0f) ? -1.0f : t2);
        pitch = std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0f * (orien._w * orien._z + orien._x * orien._y);
        double t4 = +1.0f - 2.0f * (ysqr + orien._z * orien._z);
        yaw = std::atan2(t3, t4);
        //heading = yaw;
        //printf("->Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
        //printf("->Heading: { %f %f}", orien._z, orien._w);
        oo.x = pitch;
        oo.y = roll;
        oo.z = yaw; // // 0, 360);
        return oo;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return vector3D_t(0,0,0);
}
