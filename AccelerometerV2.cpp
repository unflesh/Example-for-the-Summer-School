#include "AccelerometerV2.h"

namespace IntroSatLib {

AccelerometerV2::AccelerometerV2(TwoWire &hi2c, uint8_t address): BaseDevice(hi2c, address)
{
}

uint8_t AccelerometerV2::Init()
{
	_i2c.write(16, (uint8_t*)&ACCEL_CONFIG, 1);
	delay(100);
	return 0;
}

int16_t AccelerometerV2::RawX()
{
	uint8_t buf[2];
	_i2c.read(ACCEL_XOUT_L, buf, 2);
	delay(10);
	return buf[1] << 8 | buf[0];
}
int16_t AccelerometerV2::RawY()
{
	uint8_t buf[2];
	_i2c.read(ACCEL_YOUT_L, buf, 2);
	delay(10);
	return buf[1] << 8 | buf[0];
}
int16_t AccelerometerV2::RawZ()
{
	uint8_t buf[2];
	_i2c.read(ACCEL_ZOUT_L, buf, 2);
	delay(10);
	return buf[1] << 8 | buf[0];
}

float AccelerometerV2::X()
{
	float e = RawX() / _rawg;
	return e;
}
float AccelerometerV2::Y()
{
	float e = RawY() / _rawg;
	return e;
}
float AccelerometerV2::Z()
{
	float e = RawZ() / _rawg;
	return e;
}

AccelerometerV2::~AccelerometerV2()
{
	BaseDevice::~BaseDevice();
}

}
