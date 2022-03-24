/*
 * proximity_sensors.hpp
 *
 *  Created on: Mar 21, 2022
 *      Author: george
 */

#ifndef INC_TOF_SENSOR_HPP_
#define INC_TOF_SENSOR_HPP_

#include "hal_wrapper.hpp"

#include <array>
#include <string.h>

enum tof
{	//TODO names
	S1,
	S2,
	S3,
	S4,
	S5,
	S6,
	S7,
	COUNT
};

struct tof_sensor
{
	using value_type = uint16_t;
	static const value_type invalid_value;

	tof_sensor() :
		_value(invalid_value) {}

	void _new_val(value_type val) {_value = val;}

	value_type value()
	{
		value_type v = (value_type)_value;
		if (v > 0 && v < 3000)
			return v;
		else
			return invalid_value;
	}

private:
	volatile value_type _value;
};

const tof_sensor::value_type tof_sensor::invalid_value = UINT16_MAX;


#define TOF_PACKET_SIZE 11

class tof_sensor_manager: public std::array<tof_sensor, tof::COUNT>
{

	struct __attribute__ ((__packed__)) tof_packet {
		char start;
		char index[2];
		char sign;
		char data[6];
	};

public:

	void isr_rx_done(UART_HandleTypeDef *huart)
	{
		const int i = 0;
		if (_tof_buffer[i] == '#')
		{
			if (_tof_buffer[i + 3] == '=' && _tof_buffer[i + 10] == '\n')
			{
				tof_packet * p = (tof_packet *)_tof_buffer;
				size_t index = atoi(p->index);
				if (index < tof::COUNT)
				{
					tof_sensor::value_type val = atoi(p->data);
					at(index)._new_val(val);
				}
			}
		}
		start_receive(huart);
	}

	void start_receive(UART_HandleTypeDef *huart)
	{
		memset(_tof_buffer, 0, sizeof(_tof_buffer));
		HAL_UART_Receive_IT(huart, _tof_buffer, TOF_PACKET_SIZE);
	}

private:

	uint8_t _tof_buffer[TOF_PACKET_SIZE + 3];
};


#endif /* INC_TOF_SENSOR_HPP_ */
