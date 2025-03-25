#pragma once

#include <unistd.h>


typedef struct __euler
{
	float roll, pitch, yaw;
} Euler_t;


class Quaternion_t
{
public:
	
	float w, x, y, z;
	
public:
	
	Quaternion_t() {};
	Euler_t getEuler();
	void fromEuler(Euler_t *euler);
	void normalize(void);
	void conjugate(void);
	void add(Quaternion_t *q);
	void multiply(Quaternion_t *q);
	Quaternion_t operator+(Quaternion_t q1);
	Quaternion_t operator*(Quaternion_t q1);
};


Euler_t Quaternion_t::getEuler()
{
//	Euler_t euler;
//    euler.roll = (atan2f(2.0f*(w*x + y*z), w*w - x*x - y*y + z*z));//*180/3.1428;
//	euler.pitch = (-asinf(2.0f * (x*z - y*y)));//*180/3.1428;
//	euler.yaw = (atan2f(2.0f * (x*y + w*z), w*w + x*x - y*y - z*z));//*180/3.1428;
//	return euler;
	
	Euler_t angles;
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = sqrtf(1 + 2 * (w * y - x * z));
    float cosp = sqrtf(1 - 2 * (w * y - x * z));
    angles.pitch = 2 * atan2f(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return angles;
}

void Quaternion_t::fromEuler(Euler_t *euler)
{
	// Abbreviations for the various angular functions
    float cr = cosf(euler->roll * 0.5);
    float sr = sinf(euler->roll * 0.5);
    float cp = cosf(euler->pitch * 0.5);
    float sp = sinf(euler->pitch * 0.5);
    float cy = cosf(euler->yaw * 0.5);
    float sy = sinf(euler->yaw * 0.5);
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

void Quaternion_t::normalize()
{
	float norm = sqrtf(x*x + y*y + z*z + w*w);
	w /= norm;
	x /= norm;
	y /= norm;
	z /= norm;
}

void Quaternion_t::conjugate()
{
	x = -x;
	y = -y;
	z = -z;
}

void Quaternion_t::add(Quaternion_t *q)
{
	w += q->w;
	x += q->x;
	y += q->y;
	z += q->z;
}

Quaternion_t Quaternion_t::operator+(Quaternion_t q1)
{
	Quaternion_t q;
	q = q1;
	q.add(this);
	return q;
}

void Quaternion_t::multiply(Quaternion_t *q)
{
    w = w * q->w - x * q->x - y * q->y - z * q->z;
    x = w * q->x + x * q->w + y * q->z - z * q->y;
    y = w * q->y - x * q->z + y * q->w + z * q->x;
    z = w * q->z + x * q->y - y * q->x + z * q->w;
}

Quaternion_t Quaternion_t::operator*(Quaternion_t q1)
{
	Quaternion_t q;
	q.w = w;
	q.x = x;
	q.y = y;
	q.z = z;
	q.multiply(&q1);
	return q;
}