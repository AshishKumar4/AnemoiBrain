#pragma once 
#ifndef COMMON_H
#define COMMON_H


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
    GeoPoint_t()
    {
        
    }
};

#endif