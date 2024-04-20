#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "I2CDevice.h"
#include "BaseDevice.h"

namespace IntroSatLib {


class AccelerometerV2: public BaseDevice {

private:

	static const uint8_t BASE_ADDRESS = 0x6B;
	static constexpr float _rawg = 16384.0;

	uint8_t ACCEL_CONFIG = 0x42;
	uint8_t ACCEL_XOUT_L = 0x28;
	uint8_t ACCEL_YOUT_L = 0x2A;
	uint8_t ACCEL_ZOUT_L = 0x2C;


protected:
public:

	AccelerometerV2(TwoWire &hi2c, uint8_t address = BASE_ADDRESS);

	uint8_t Init();

	int16_t RawX();
	int16_t RawY();
	int16_t RawZ();

	float X();
	float Y();
	float Z();

	virtual ~AccelerometerV2();
};

} /* namespace IntroSatLib */

#endif /* ACCELEROMETER_H_ */
