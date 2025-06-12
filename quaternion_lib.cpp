#include "quaternion_lib.h"

#define QUATERNION_LIB_ATTR		



void Euler_t::add(const Euler_t& euler)
{
	roll += euler.roll;
	pitch += euler.pitch;
	yaw += euler.yaw;
}

void Euler_t::sub(const Euler_t& euler)
{
	roll -= euler.roll;
	pitch -= euler.pitch;
	yaw -= euler.yaw;
}

void Euler_t::mul(const float val)
{
	roll *= val;
	pitch *= val;
	yaw *= val;
}

Euler_t Euler_t::operator+(const Euler_t& euler) const
{
	Euler_t e = *this;
	e.add(euler);
	return e;
}

Euler_t Euler_t::operator-(const Euler_t& euler) const
{
	Euler_t e = *this;
	e.sub(euler);
	return e;
}

Euler_t Euler_t::operator*(const float val) const
{
	Euler_t e = *this;
	e.mul(val);
	return e;
}

Euler_t& Euler_t::operator+=(const Euler_t& euler)
{
	add(euler);
	return *this;
}

Euler_t& Euler_t::operator-=(const Euler_t& euler)
{
	sub(euler);
	return *this;
}

Euler_t& Euler_t::operator*=(const float val)
{
	mul(val);
	return *this;
}


Euler_t QUATERNION_LIB_ATTR Quaternion_t::getEuler()
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
    angles.set_roll(atan2f(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    float sinp = sqrtf(1 + 2 * (w * y - x * z));
    float cosp = sqrtf(1 - 2 * (w * y - x * z));
    angles.set_pitch(2 * atan2f(sinp, cosp) - M_PI / 2);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.set_yaw(atan2f(siny_cosp, cosy_cosp));

    return angles;
}

void QUATERNION_LIB_ATTR Quaternion_t::fromEuler(Euler_t& euler)
{
	// Abbreviations for the various angular functions
    float cr = cosf(euler.get_roll() * 0.5);
    float sr = sinf(euler.get_roll() * 0.5);
    float cp = cosf(euler.get_pitch() * 0.5);
    float sp = sinf(euler.get_pitch() * 0.5);
    float cy = cosf(euler.get_yaw() * 0.5);
    float sy = sinf(euler.get_yaw() * 0.5);
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

void QUATERNION_LIB_ATTR Quaternion_t::normalize()
{
	float norm = sqrtf(x*x + y*y + z*z + w*w);
	w /= norm;
	x /= norm;
	y /= norm;
	z /= norm;
}

Quaternion_t QUATERNION_LIB_ATTR Quaternion_t::conjugate()
{
	Quaternion_t res = *this;
	res.x = -x;
	res.y = -y;
	res.z = -z;
	return res;
}

void QUATERNION_LIB_ATTR Quaternion_t::add(const Quaternion_t& q)
{
	w += q.w;
	x += q.x;
	y += q.y;
	z += q.z;
}

void QUATERNION_LIB_ATTR Quaternion_t::multiply(const Quaternion_t& q)
{
    w = w * q.w - x * q.x - y * q.y - z * q.z;
    x = w * q.x + x * q.w + y * q.z - z * q.y;
    y = w * q.y - x * q.z + y * q.w + z * q.x;
    z = w * q.z + x * q.y - y * q.x + z * q.w;
}

Quaternion_t QUATERNION_LIB_ATTR Quaternion_t::operator+(const Quaternion_t& q1)
{
	Quaternion_t q = *this;
	q.add(q1);
	return q;
}

Quaternion_t QUATERNION_LIB_ATTR Quaternion_t::operator*(const Quaternion_t& q1)
{
	Quaternion_t q = *this;
	q.multiply(q1);
	return q;
}

Quaternion_t& QUATERNION_LIB_ATTR Quaternion_t::operator+=(const Quaternion_t& q)
{
	add(q);
	return *this;
}

Quaternion_t& QUATERNION_LIB_ATTR Quaternion_t::operator*=(const Quaternion_t& q)
{
	multiply(q);
	return *this;
}

XYZ_t XYZ_t::operator+(const XYZ_t& vect) const
{
	XYZ_t res = *this;
	res.x += vect.x;
	res.y += vect.y;
	res.z += vect.z;
	return res;
}

XYZ_t XYZ_t::operator-(const XYZ_t& vect) const
{
	XYZ_t res = *this;
	res.x -= vect.x;
	res.y -= vect.y;
	res.z -= vect.z;
	return res;
}

XYZ_t XYZ_t::operator*(const float val) const
{
	XYZ_t res = *this;
	res.x *= val;
	res.y *= val;
	res.z *= val;
	return res;
}

XYZ_t& XYZ_t::operator+=(const XYZ_t& vect)
{
	x += vect.x;
	y += vect.y;
	z += vect.z;
	return *this;
}

XYZ_t& XYZ_t::operator-=(const XYZ_t& vect)
{
	x -= vect.x;
	y -= vect.y;
	z -= vect.z;
	return *this;
}

XYZ_t& XYZ_t::operator*=(const float val)
{
	x *= val;
	y *= val;
	z *= val;
	return *this;
}

XYZ_t XYZ_t::rotate(Quaternion_t& q)
{
	Quaternion_t this_vect(0, x, y, z);
	Quaternion_t res;
	res = q * this_vect;
	res = res * q.conjugate();
	return XYZ_t(res.x, res.y, res.z);
}