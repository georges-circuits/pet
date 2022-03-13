
#define SP_NO_IOSTREAM

#include "init.h"

#include "../libprotoserial/libprotoserial/interface.hpp"
#include "../libprotoserial/libprotoserial/fragmentation.hpp"
#include "../libprotoserial/libprotoserial/clock.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace sp::literals;

class servo
{
public:

	using pulse_duration = sp::clock::duration;

	/* this assumes that the timer is setup to generate 50Hz PWM */
	servo(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t counter_period):
		_htim(htim), _channel(channel), _counter_period(counter_period)
	{
		set_pulse_min(1ms);
		set_pulse_max(2ms);
		__HAL_TIM_SET_COMPARE(_htim, _channel, counter_mid());
		HAL_TIM_PWM_Start(_htim, _channel);
	}

	/* value from 0.0 to 1.0 */
	void set(float deflection)
	{
		if (deflection < 0.0) deflection = 0.0;
		else if (deflection > 1.0) deflection = 1.0;
		__HAL_TIM_SET_COMPARE(_htim, _channel, (uint32_t)(1.0 *
				(_counter_max - _counter_min) * deflection + 1.0 * _counter_min));
	}

	void set_pulse_min(pulse_duration duration) {_counter_min = duration_to_tim(duration);}
	void set_pulse_max(pulse_duration duration) {_counter_max = duration_to_tim(duration);}

private:

	uint32_t counter_mid() const {return (_counter_min + _counter_max) / 2.0;}
	uint32_t duration_to_tim(pulse_duration duration) const {
		return (uint32_t)(1.0 * (duration / _period_duration) * _counter_period);
	}

	TIM_HandleTypeDef *_htim;
	uint32_t _channel;
	uint32_t _counter_period, _counter_min, _counter_max;
	const pulse_duration _period_duration = 20ms;
};


UART_HandleTypeDef *uart0_huart = &huart1;
sp::uart_interface *uart0_handle;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == uart0_huart->Instance)
		uart0_handle->isr_rx_done();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == uart0_huart->Instance)
		uart0_handle->isr_tx_done();
}



int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();

	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	//servo s(&htim3, TIM_CHANNEL_1, 19999);


	//sp::loopback_interface interface(0, 1, 10, 64, 256);
	sp::uart_interface uart0_interface(uart0_huart, 0, 1, 10, 64, 512);
	uart0_handle = &uart0_interface;
	sp::fragmentation_handler uart0_handler(uart0_interface.max_data_size(), 10ms, 100ms, 2);

	while (1)
	{
		uart0_interface.main_task();
		uart0_handler.main_task();

		/*for (float d = 0.0; d < 1.0; d += 0.05)
		{
			s.set(d);
			HAL_Delay(100);
		}*/

	}
}




