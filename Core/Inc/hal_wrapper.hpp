/*
 * hal_wrapper.hpp
 *
 *  Created on: Mar 15, 2022
 *      Author: george
 */

#ifndef INC_HAL_WRAPPER_HPP_
#define INC_HAL_WRAPPER_HPP_

#if defined(STM32L073xx) || defined(STM32L031xx)
#include "stm32l0xx_hal.h"
#else
#error "unknown device"
#endif

namespace stm32
{
#ifdef HAL_TIM_MODULE_ENABLED
	class timer
	{
	protected:
		TIM_HandleTypeDef *_htim;
		uint32_t _prescaler, _period;

	public:
		timer(TIM_HandleTypeDef *htim, uint32_t prescaler, uint32_t period):
			_htim(htim), _prescaler(prescaler), _period(period) {}

		void start_base_it() {HAL_TIM_Base_Start_IT(_htim);};
		void stop_base_it() {HAL_TIM_Base_Stop_IT(_htim);};
		void start_pwm(uint32_t channel) {HAL_TIM_PWM_Start(_htim, channel);}
		void stop_pwm(uint32_t channel) {HAL_TIM_PWM_Stop(_htim, channel);}
		void start_oc(uint32_t channel) {HAL_TIM_OC_Start(_htim, channel);}
		void stop_oc(uint32_t channel) {HAL_TIM_OC_Stop(_htim, channel);}

		void set_pwm_dutycycle(uint32_t channel, uint32_t value) {__HAL_TIM_SET_COMPARE(_htim, channel, value);}

		void set_prescaler(uint32_t value) {__HAL_TIM_SET_PRESCALER(_htim, value);}
		void set_period(uint32_t value) {
			__HAL_TIM_SET_AUTORELOAD(_htim, value == 0 ? 1 : value);
			__HAL_TIM_SET_COUNTER(_htim, 0);
		}

		TIM_HandleTypeDef * handle() {return _htim;}
		uint32_t prescaler() const {return _prescaler;}
		uint32_t period() const {return _period;}
	};

	class timer_channel
	{
	protected:
		timer* _tim;
		uint32_t _channel;

	public:
		timer_channel(timer* tim, uint32_t channel):
			_tim(tim), _channel(channel) {}

		const timer & get_timer() const {return *_tim;}
	};

	class timer_oc_channel : public timer_channel
	{
	public:
		timer_oc_channel(timer* tim, uint32_t channel):
			timer_channel(tim, channel) {}

		void set_period(uint32_t value) {_tim->set_period(value);}
		uint32_t period() const {return _tim->period();}

		void start() {_tim->start_oc(_channel);}
		void stop() {_tim->stop_oc(_channel);}
	};

	class timer_pwm_channel : public timer_channel
	{
	public:
		timer_pwm_channel(timer* tim, uint32_t channel):
			timer_channel(tim, channel) {}

		void set_dutycycle(uint32_t value) {_tim->set_pwm_dutycycle(_channel, value);}

		void start() {_tim->start_pwm(_channel);}
		void stop() {_tim->stop_pwm(_channel);}
	};
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
	class gpio
	{
	protected:
		GPIO_TypeDef * _port;
		uint32_t _pin;

	public:

		gpio(GPIO_TypeDef *port, uint32_t pin):
			_port(port), _pin(pin) {}

		virtual bool read() const {return (bool)(HAL_GPIO_ReadPin(_port, _pin));}
		virtual void write(bool state) {HAL_GPIO_WritePin(_port, _pin, (GPIO_PinState)(state));}
		void toggle() {HAL_GPIO_TogglePin(_port, _pin);}

		void set() {write(true);}
		void reset() {write(false);}
	};

	class gpio_inv : public gpio
	{
	protected:
		bool _inverted;

	public:

		gpio_inv(GPIO_TypeDef *port, uint32_t pin, bool inverted = false):
			gpio(port, pin), _inverted(inverted) {}

		bool read() const {return _inverted != gpio::read();}
		void write(bool state) {gpio::write(_inverted != state);}

		bool is_inverted() const {return _inverted;}
		void set_invert(bool invert) {_inverted = invert;}
	};
#endif
#ifdef HAL_SPI_MODULE_ENABLED
	class spi
	{
		SPI_HandleTypeDef *_hspi;

	public:
		spi(SPI_HandleTypeDef *hspi) :
			_hspi(hspi) {}

		bool transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout = 100)
		{
			return HAL_SPI_Transmit(_hspi, pData, Size, Timeout) == HAL_OK;
		}
		bool receive(uint8_t *pData, uint16_t Size, uint32_t Timeout = 100)
		{
			return HAL_SPI_Receive(_hspi, pData, Size, Timeout) == HAL_OK;
		}
		bool transmitreceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout = 100)
		{
			return HAL_SPI_TransmitReceive(_hspi, pTxData, pRxData, Size, Timeout) == HAL_OK;
		}
	};
#endif
#ifdef HAL_I2C_MODULE_ENABLED
	class i2c
	{
		I2C_HandleTypeDef *_hi2c;

	public:
		i2c(I2C_HandleTypeDef *hi2c) :
			_hi2c(hi2c) {}

		bool transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout = 100)
		{
			return HAL_I2C_Master_Transmit(_hi2c, DevAddress << 1, pData, Size, Timeout) == HAL_OK;
		}
		bool receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout = 100)
		{
			return HAL_I2C_Master_Receive(_hi2c, DevAddress << 1, pData, Size, Timeout) == HAL_OK;
		}
	};
#endif
}


#endif /* INC_HAL_WRAPPER_HPP_ */
