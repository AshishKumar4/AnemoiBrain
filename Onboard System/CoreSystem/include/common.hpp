#pragma once

#include "configurations.h"
#include "iostream"
#include "vector"
#include <cmath>
#include <thread>
#include <rpc/msgpack.hpp>

// using namespace std;

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

	inline void setX(const float val) { x = val; }
	inline void setY(const float val) { y = val; }
	inline void setZ(const float val) { z = val; }

	// inline float x() const {return x;}
	// inline float y() const {return y;}
	// inline float z() const {return z;}

	const vector3D_t &operator+=(vector3D_t &vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	}

	const vector3D_t &operator-=(vector3D_t &vec)
	{
		x -= vec.x;
		y -= vec.y;
		z -= vec.z;
		return *this;
	}

	vector3D_t &operator=(const vector3D_t &vec)
	{
		if (this == &vec)
			return *this;

		x = vec.x;
		y = vec.y;
		z = vec.z;

		return *this;
	}

	float length() { return 0; }
	static float dotProduct(const vector3D_t &a, const vector3D_t &b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	static void crossProduct(const vector3D_t &a, const vector3D_t &b, vector3D_t &d)
	{
		d.set((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
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

	quaternion_t(vector3D_t &vec)
	{
		fromEuler(vec);
	}

	void set(float w, float x, float y, float z)
	{
		this->_w = w;
		this->_x = x;
		this->_y = y;
		this->_z = z;
	}

	quaternion_t &operator+=(const quaternion_t &quat)
	{
		this->_w += quat._w;
		this->_x += quat._x;
		this->_y += quat._y;
		this->_z += quat._z;

		return *this;
	}

	quaternion_t &operator-=(const quaternion_t &quat)
	{
		this->_w -= quat._w;
		this->_x -= quat._x;
		this->_y -= quat._y;
		this->_z -= quat._z;

		return *this;
	}

	quaternion_t &operator*=(const quaternion_t &qb)
	{
		quaternion_t qa;

		qa = *this;

		_w = qa.w() * qb.w() - qa.x() * qb.x() - qa.y() * qb.y() - qa.z() * qb.z();
		_x = qa.w() * qb.x() + qa.x() * qb.w() + qa.y() * qb.z() - qa.z() * qb.y();
		_y = qa.w() * qb.y() - qa.x() * qb.z() + qa.y() * qb.w() + qa.z() * qb.x();
		_z = qa.w() * qb.z() + qa.x() * qb.y() - qa.y() * qb.x() + qa.z() * qb.w();

		return *this;
	}
	quaternion_t &operator*=(const float val)
	{
		this->_w *= val;
		this->_x *= val;
		this->_y *= val;
		this->_z *= val;

		return *this;
	}

	quaternion_t &operator-=(const float val)
	{
		this->_w -= val;
		this->_x -= val;
		this->_y -= val;
		this->_z -= val;

		return *this;
	}

	quaternion_t &operator=(const quaternion_t &quat)
	{
		if (this == &quat)
			return *this;

		this->_w = quat._w;
		this->_x = quat._x;
		this->_y = quat._y;
		this->_z = quat._z;

		return *this;
	}

	quaternion_t &operator=(const vector3D_t &vec)
	{
		fromEuler(vec);

		return *this;
	}

	const quaternion_t operator*(const quaternion_t &qb) const
	{
		quaternion_t result = *this;
		result *= qb;
		return result;
	}

	const quaternion_t operator*(const float val) const
	{
		quaternion_t result = *this;
		result *= val;
		return result;
	}

	const quaternion_t operator-(const quaternion_t &qb) const
	{
		quaternion_t result = *this;
		result -= qb;
		return result;
	}

	const quaternion_t operator-(const float val) const
	{
		quaternion_t result = *this;
		result -= val;
		return result;
	}

	quaternion_t conjugate() const
	{
		quaternion_t q;
		q.setW(_w);
		q.setX(-_x);
		q.setY(-_y);
		q.setZ(-_z);
		return q;
	}

	void toAngleVector(float &angle, vector3D_t &vec)
	{
		const float halfTheta = acos(_w);
		const float sinHalfTheta = sin(halfTheta);

		if (sinHalfTheta == 0) 
		{
			vec.setX(1.0);
			vec.setY(0);
			vec.setZ(0);
		} 
		else 
		{
			vec.setX(_x / sinHalfTheta);
			vec.setY(_x / sinHalfTheta);
			vec.setZ(_x / sinHalfTheta);
		}
		angle = 2.0 * halfTheta;
	}

	void fromAngleVector(const float &angle, const vector3D_t &vec)
	{
		const float sinHalfTheta = sin(angle / 2.0);
		_w = cos(angle / 2.0);
		_x = vec.x * sinHalfTheta;
		_y = vec.y * sinHalfTheta;
		_z = vec.z * sinHalfTheta;
	}


	void normalize()
	{
		const float length = sqrt(_w * _w + _x * _x +
				_y * _y + _z * _z);

		if ((length == 0) || (length == 1))
			return;

		_w /= length;
		_x /= length;
		_y /= length;
		_z /= length;
	}	

	void toEuler(vector3D_t &vec)
	{
    	const float x = (atan2(2.0 * (_y * _z + _w * _x),
            1 - 2.0 * (_x * _x + _y * _y)));

		const float y = (asin(2.0 * (_w * _y - _x * _z)));

		const float z = (atan2(2.0 * (_x * _y + _w * _z),
				1 - 2.0 * (_y * _y + _z * _z)));
		vec.set(x, y, z);
	}

	vector3D_t toEuler()
	{
		vector3D_t vec;
		toEuler(vec);
		return vec;
	}

	void fromEuler(const vector3D_t &vec)
	{
		const float cosX2 = cos(vec.x / 2.0f);
		const float sinX2 = sin(vec.x / 2.0f);
		const float cosY2 = cos(vec.y / 2.0f);
		const float sinY2 = sin(vec.y / 2.0f);
		const float cosZ2 = cos(vec.z / 2.0f);
		const float sinZ2 = sin(vec.z / 2.0f);

		_w = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
		_x = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
		_y = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
		_z = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
		normalize();
	}

	inline float w() const { return _w; }
	inline float x() const { return _x; }
	inline float y() const { return _y; }
	inline float z() const { return _z; }

	inline void setW(const float val) { _w = val; }
	inline void setX(const float val) { _x = val; }
	inline void setY(const float val) { _y = val; }
	inline void setZ(const float val) { _z = val; }
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

class GeoCoords_t
{
	float latitude;
	float longitude;

public:
	GeoCoords_t(float lat, float lon) : latitude(lat), longitude(lon)
	{
	}

	GeoCoords_t() : latitude(0), longitude(0)
	{
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
	std::vector<uint8_t> buff;
	MSGPACK_DEFINE_MAP(buff);
};

class DroneState_t
{
public:
	data_imu_t imu;
	vector3D_t vel;
	float altitude;
	float heading; // in Radians
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

#define PI 3.14159265358979323846
#define CONST_PIby180 PI / 180
#define CONST_180byPI 180 / PI

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
	if (angle > 180)
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
	return vector3D_t(0, 0, 0);
}
