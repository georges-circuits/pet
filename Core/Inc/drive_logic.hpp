/*
 * drive_logic.hpp
 *
 *  Created on: Apr 12, 2022
 *      Author: george
 */

#ifndef INC_DRIVE_LOGIC_HPP_
#define INC_DRIVE_LOGIC_HPP_

#include "moves.hpp"
#include "clock.hpp"


struct drive_output
{
	float drive, steering;

	drive_output(float drive, float steering) :
		drive(drive), steering(steering) {}

	drive_output() :
		drive_output(0, 0) {}
};


class drive_const : public move<drive_output>
{
protected:
	drive_output _out;
	std::function<bool(void)> _stop;

	virtual status do_update() {return NO_UPDATE;}

public:
	drive_const(float speed, float steer, std::function<bool(void)> stop):
		_out(speed, steer), _stop(std::move(stop)) {}

	status update()
	{
		if (_stop())
			return FINISHED;

		return do_update();
	}
	drive_output get() {return _out;}
};


class drive_pivot : public drive_const
{
	clock::duration _tick;
	clock::time_point _last;

	status do_update()
	{
		if (_tick + _last < clock::now())
		{
			_last = clock::now();
			_out = drive_output(-_out.drive, -_out.steering);
			return UPDATED;
		}

		return NO_UPDATE;
	}

public:
	drive_pivot(float speed, float steer, clock::duration tick, std::function<bool(void)> stop):
		drive_const(speed, steer, std::move(stop)), _tick(tick), _last(clock::now()) {}
};



#endif /* INC_DRIVE_LOGIC_HPP_ */
