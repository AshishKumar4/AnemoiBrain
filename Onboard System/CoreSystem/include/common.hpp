#pragma once 
#ifndef COMMON_H
#define COMMON_H

#include "thread"
#include "iostream"

class vector3D_t
{
public:
    float x;
    float y;
    float z;

    vector3D_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    
    vector3D_t(float xval, float yval, float zval)
    {
        x = xval;
        y = yval; 
        z = zval;
    }
};

class quaternion_t 
{
    float _w;
    float _x;
    float _y;
    float _z;
public: 

    quaternion_t()
    {
        _w = 1;
        _x = 0;
        _y = 0;
        _z = 0;
    }

    quaternion_t(float w, float x, float y, float z)
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

    GeoPoint_t()
    {
        x = 0;
        y = 0;
        z = 0;
    }
    
    GeoPoint_t(float xval, float yval, float zval)
    {
        x = xval;
        y = yval; 
        z = zval;
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

inline float degreesToRads(float degree)
{
    return (degree/180)*PI;
}

inline float radsToDegrees(float degree)
{
    return (degree/PI)*180;
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

#endif