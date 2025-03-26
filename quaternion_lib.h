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



typedef struct __euler
{
	float roll, pitch, yaw;
} Euler_t;

typedef struct
{
	float x, y, z;
} XYZ_t;


class Quaternion_t
{
public:
	
	float w, x, y, z;
	
public:
	
	Quaternion_t() {};
	Quaternion_t(float _w, float _x, float _y, float _z)
		: w {_w}
		, x {_x}
		, y {_y}
		, z {_z}
		{}
	Euler_t getEuler();
	void fromEuler(Euler_t *euler);
	void normalize(void);
	void conjugate(void);
	void add(Quaternion_t *q);
	void multiply(Quaternion_t *q);
	Quaternion_t operator+(Quaternion_t q1);
	Quaternion_t operator*(Quaternion_t q1);
	XYZ_t rotateVect(XYZ_t *v);
};