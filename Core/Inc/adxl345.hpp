/*
 * adxl345.hpp
 *
 *  Created on: Apr 12, 2022
 *      Author: george
 */

#ifndef INC_ADXL345_HPP_
#define INC_ADXL345_HPP_

#include "hal_wrapper.hpp"

class ADXL345
{
	stm32::spi *_spi;
	stm32::gpio _ncs;

	enum flag : uint8_t
	{
		READ = 0x80,
		WRITE = 0x00,
		MULTIBYTE = 0x40
	};

	enum reg : uint8_t
	{
		DATAX0 = 0x32,
		DATA_BASE_ = DATAX0,
		POWER_CTRL = 0x2D
	};

	enum bit : uint8_t
	{
		MEASURE = 0x08
	};

	void enable() {_ncs.reset();}
	void disable() {_ncs.set();}

public:

	struct data
	{
		int16_t x, y, z;
	};

	ADXL345(stm32::spi *spi, stm32::gpio ncs):
		_spi(spi), _ncs(ncs)
	{
		uint8_t writeData[2];
		writeData[0] = (flag::WRITE | reg::POWER_CTRL);
		writeData[1] = bit::MEASURE;

		enable();
		_spi->transmit(writeData, sizeof(writeData));
		disable();
	}

	data read()
	{
		data ret;
		uint8_t writeData[1 + 3 * 2] = {0};
		uint8_t readData[sizeof(writeData)] = {0};

		enable();
		writeData[0] = (flag::READ | flag::MULTIBYTE | reg::DATA_BASE_);
		_spi->transmitreceive(writeData, readData, sizeof(writeData), 100);
		disable();

		ret.x = (((int16_t)readData[1]) | (((int16_t)readData[2]) << 8));
		ret.y = (((int16_t)readData[3]) | (((int16_t)readData[4]) << 8));
		ret.z = (((int16_t)readData[5]) | (((int16_t)readData[6]) << 8));

		return ret;
	}
};



#endif /* INC_ADXL345_HPP_ */
