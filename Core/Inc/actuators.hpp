/*
 * actuators.hpp
 *
 *  Created on: Mar 15, 2022
 *      Author: george
 */

#ifndef INC_ACTUATORS_HPP_
#define INC_ACTUATORS_HPP_

#include "hal_wrapper.hpp"

#include <memory>
#include <utility>
#include <cmath>
#include <list>

template<typename val_type>
constexpr bool is_within(val_type value, val_type pivot, val_type range)
{
	return value >= pivot - range && value <= pivot + range;
}

template<typename val_type>
struct actuator_calibrator
{
	virtual val_type forward(val_type) = 0;
};

template<typename val_type>
struct actuator_filter
{
	virtual val_type forward(val_type) = 0;
};

template<typename val_type>
class filter_linear : public actuator_filter<val_type>
{
	const val_type _increment;
	val_type _current;

public:

	filter_linear(val_type increment, val_type initial = 0):
		_increment(increment), _current(initial) {}

	val_type forward(val_type setpoint)
	{
		if (is_within(_current, setpoint, _increment))
			return _current = setpoint;
		else
		{
			if (setpoint > _current)
				return _current += _increment;
			else
				return _current -= _increment;
		}
	}
};

template<typename val_type>
class actuator
{
	std::unique_ptr<actuator_calibrator<val_type>> _calibrator;
	std::unique_ptr<actuator_filter<val_type>> _filter;

	val_type _setpoint, _current;

	virtual void do_set(float) = 0;

public:

	void set(val_type setpoint)
	{
		if (setpoint < setpoint_min()) setpoint = setpoint_min();
		else if (setpoint > setpoint_max()) setpoint = setpoint_max();
		_setpoint = setpoint;
		update();
	}

	void update()
	{
		val_type s = _setpoint;
		if (_filter) s = _filter->forward(s);
		_current = s;
		if (_calibrator) s = _calibrator->forward(s);
		do_set(s);
	}

	val_type get_current() const {return _current;}
	val_type get_setpoint() const {return _setpoint;}

	virtual val_type setpoint_max() const = 0;
	virtual val_type setpoint_min() const = 0;

	void set_calibrator(std::unique_ptr<actuator_calibrator<val_type>> calibrator) {_calibrator = std::move(calibrator);}
	void set_filter(std::unique_ptr<actuator_filter<val_type>> filter) {_filter = std::move(filter);}
};

template<class actuator_type>
class actuator_manager
{
	std::list<std::unique_ptr<actuator_type>> _actuators;

public:

	template<class Type, typename... Args>
	actuator_type* new_actuator(Args... args)
	{
		auto ret = new Type(std::forward<Args>(args)...);
		_actuators.push_back(std::unique_ptr<actuator_type>(ret));
		return ret;
	}

	void update()
	{
		for (auto & a : _actuators)
		{
			if (a->get_current() != a->get_setpoint())
				a->update();
		}
	}

};




class test_actuator : public actuator<float>
{
	void do_set(float deflection) {}

public:
	test_actuator() {}

	float setpoint_max() const {return 1.0;}
	float setpoint_min() const {return -1.0;}
};

class servo : public actuator<float>
{
	stm32::timer_pwm_channel _pwm;
	uint32_t _counter_min, _counter_max;

	void do_set(float deflection)
	{
		_pwm.set_dutycycle((uint32_t)(((_counter_max - _counter_min)
				* ((deflection + 1.0) / 2.0)) + _counter_min));
	}

public:
	/* this assumes that the timer is setup to generate 50Hz PWM */
	servo(stm32::timer_pwm_channel pwm):
		_pwm(std::move(pwm))
	{
		_counter_min = ((1.0 / 20.0) * _pwm.get_timer().period());
		_counter_max = ((2.0 / 20.0) * _pwm.get_timer().period());

		_pwm.start();
		set(0.0);
	}

	float setpoint_max() const {return 1.0;}
	float setpoint_min() const {return -1.0;}
};

class simple_stepper : public actuator<float>
{
protected:
	const float _max = 1.0, _min = -1.0;//, _deadband = 0.001;

	stm32::timer_oc_channel _step_gen;
	stm32::gpio_inv _direction, _enable;
	bool _brake, _started = false;

	void do_set(float ratio)
	{
		if (ratio == 0.0)
		{
			stop();
		}
		else //if (ratio >= _deadband || ratio <= _deadband)
		{
			if (ratio > 0.0)
				_direction.set();
			else
			{
				_direction.reset();
				ratio = -ratio;
			}
			/* now the ratio belongs to the interval (_deadband,1], we want to map
			 * this to the interval [1, _step_gen.period()] using inverse function */
			/*_step_gen.set_period((uint32_t)((ratio + _deadband >= 1.0) ?
					1.0 : (_deadband / ratio) * (float)_step_gen.period())
			);*/

			uint32_t max = _step_gen.period(), min = max / 50;
			uint32_t p = -std::log10((ratio + 0.1) / 1.1) * (float)max;
			_step_gen.set_period(p > max ? max : (p < min ? min : p));

			start();
		}
	}

	void start()
	{
		if (!_started)
		{
			_started = true;
			_step_gen.start();
			_enable.set();
		}
	}

	void stop()
	{
		if (_started)
		{
			_started = false;
			_step_gen.stop();
			if (!_brake)
				_enable.reset();
		}
	}

public:

	simple_stepper(stm32::timer_oc_channel step_generator, stm32::gpio_inv direction,
			stm32::gpio_inv enable, bool brake = true) :
		_step_gen(step_generator), _direction(direction), _enable(enable), _brake(brake)
	{
		set(0);
	}

	float setpoint_max() const {return _max;}
	float setpoint_min() const {return _min;}
};






#endif /* INC_ACTUATORS_HPP_ */
