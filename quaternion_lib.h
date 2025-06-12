#pragma once

#include <cmath>
using namespace std;

#define __QUATERNION_LIB_LOCATE_RAM

#define __QUATERNION_LIB_ESP32
//#define __QUATERNION_LIB_MIK32

#if defined(__QUATERNION_LIB_LOCATE_RAM)
#if defined(__QUATERNION_LIB_ESP32)
#define QUATERNION_LIB_ATTR		IRAM_ATTR
#else
#if defined(__QUATERNION_LIB_MIK32)
#define QUATERNION_LIB_ATTR		RAM_ATTR
#endif
#endif
#endif



//typedef struct __euler
//{
//	float roll, pitch, yaw;
//} Euler_t;
class Euler_t
{
	float roll=0, pitch=0, yaw=0;
	
public:
	Euler_t() {};
	Euler_t(float _r, float _p, float _y)
		: roll(_r)
		, pitch(_p)
		, yaw(_y)
		{}
	float get_roll() const {return roll;}
	float get_pitch() const {return pitch;}
	float get_yaw() const {return yaw;}
	void set_roll(const float _r) {roll = _r;}
	void set_pitch(const float _p) {pitch = _p;}
	void set_yaw(const float _y) {yaw = _y;}
	void add(const Euler_t& euler);
	void sub(const Euler_t& euler);
	void mul(const float val);
	Euler_t operator+(const Euler_t& euler) const;
	Euler_t operator-(const Euler_t& euler) const;
	Euler_t operator*(const float val) const;
	Euler_t& operator+=(const Euler_t& euler);
	Euler_t& operator-=(const Euler_t& euler);
	Euler_t& operator*=(const float val);
};


class Quaternion_t
{
public:
	
	float w = 1, x = 0, y = 0, z = 0;
	
public:
	
	Quaternion_t() {};
	Quaternion_t(float _w, float _x, float _y, float _z)
		: w {_w}
		, x {_x}
		, y {_y}
		, z {_z}
		{}
	Quaternion_t(Euler_t& from_euler)
	{
		fromEuler(from_euler);
	}
	Euler_t getEuler();
	void fromEuler(Euler_t& euler);
	void normalize(void);
	Quaternion_t conjugate(void);
	void add(const Quaternion_t& q);
	void multiply(const Quaternion_t& q);
	Quaternion_t operator+(const Quaternion_t& q1);
	Quaternion_t operator*(const Quaternion_t& q1);
	Quaternion_t& operator+=(const Quaternion_t& q);
	Quaternion_t& operator*=(const Quaternion_t& q);
};


struct XYZ_t
{
	float x, y, z;
	XYZ_t() = default;
	XYZ_t(float _x, float _y, float _z)
		: x(_x)
		, y(_y)
		, z(_z)
	{}
	XYZ_t operator+(const XYZ_t& vect) const;
	XYZ_t operator-(const XYZ_t& vect) const;
	XYZ_t operator*(const float val) const;
	XYZ_t& operator+=(const XYZ_t& vect);
	XYZ_t& operator-=(const XYZ_t& vect);
	XYZ_t& operator*=(const float val);
	XYZ_t rotate(Quaternion_t& q);
};