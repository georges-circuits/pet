
#define SP_NO_IOSTREAM

#include "init.h"

#include "../libprotoserial/libprotoserial/interface.hpp"
#include "../libprotoserial/libprotoserial/fragmentation.hpp"
#include "../libprotoserial/libprotoserial/clock.hpp"

#include <chrono>

using namespace std::chrono_literals;
using namespace sp::literals;

#include "hal_wrapper.hpp"
#include "actuators.hpp"




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
	MX_TIM3_Init();

	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	stm32::timer tim3(&htim3, 31, 9999);
	stm32::timer tim2(&htim2, 319, 999);

	servo steering(stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1));
	simple_stepper drive(
			stm32::timer_oc_channel(&tim2, TIM_CHANNEL_1),
			stm32::gpio_inv(STEP_DIR_GPIO_Port, STEP_DIR_Pin),
			stm32::gpio_inv(STEP_EN_GPIO_Port, STEP_EN_Pin)
	);

	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);

	//sp::loopback_interface interface(0, 1, 10, 64, 256);
	sp::uart_interface uart0_interface(uart0_huart, 0, 1, 10, 64, 512);
	uart0_handle = &uart0_interface;
	sp::fragmentation_handler uart0_handler(uart0_interface.max_data_size(), 10ms, 100ms, 3);

	while (1)
	{
		//uart0_interface.main_task();
		//uart0_handler.main_task();

		for (float d = 0; d <= 1.2; d += 0.02)
		{
			//steering.set(d);
			drive.set(d);
			HAL_Delay(500);
		}

	}
}




