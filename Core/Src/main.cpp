
#define SP_NO_IOSTREAM

#include "init.h"

#include "../libprotoserial/libprotoserial/interface.hpp"
#include "../libprotoserial/libprotoserial/fragmentation.hpp"
#include "../libprotoserial/libprotoserial/clock.hpp"

#include <chrono>
using namespace std::chrono_literals;

#include <stdio.h>
#include <stdarg.h>
#include "syscalls_retarget.h"

#include "hal_wrapper.hpp"
#include "actuators.hpp"
#include "tof_sensor.hpp"



UART_HandleTypeDef *uart0_huart = &huart1;
sp::uart_interface *uart0_handle;
tof_sensor_manager *tof_sensors_handle;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == uart0_huart->Instance)
		uart0_handle->isr_rx_done();
	else if (huart->Instance == huart4.Instance || huart->Instance == huart5.Instance)
		tof_sensors_handle->isr_rx_done(huart);

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == uart0_huart->Instance)
		uart0_handle->isr_tx_done();
}

void Debug_Print(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();

	//RetargetInit(&huart2);

	//Debug_Print("Hi\n");

	stm32::timer tim2(&htim2, 6, 999);
	stm32::timer tim3(&htim3, 31, 19999);

	servo steering(stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1));
	simple_stepper drive(
			stm32::timer_oc_channel(&tim2, TIM_CHANNEL_1),
			stm32::gpio_inv(STEP_DIR_GPIO_Port, STEP_DIR_Pin, true),
			stm32::gpio_inv(STEP_EN_GPIO_Port, STEP_EN_Pin, true)
	);

	stm32::gpio_inv ir_pin(IR_GPIO_Port, IR_Pin);

	/*tof_sensor_manager tof_sensors;
	tof_sensors_handle = &tof_sensors;
	tof_sensors.start_receive(&huart4);
	tof_sensors.start_receive(&huart5);*/

	//sp::loopback_interface interface(0, 1, 10, 64, 256);
	sp::uart_interface uart0_interface(uart0_huart, 0, 1, 10, 64, 512);
	uart0_handle = &uart0_interface;
	sp::fragmentation_handler uart0_handler(uart0_interface.interface_id(), uart0_interface.max_data_size(), 50ms, 100ms, 3);
	uart0_handler.bind_to(uart0_interface);

	/*uart0_interface.receive_event.subscribe([&](sp::fragment f) {
		auto check = sp::footers::crc32(f.data());
		uart0_interface.write(sp::fragment(f.source(), std::move(f.data())));
		HAL_GPIO_TogglePin(DBG1_GPIO_Port, DBG1_Pin);
	});*/

	uart0_handler.transfer_receive_event.subscribe([&](sp::transfer t) {
		auto resp = sp::transfer(std::move(t.create_response()), sp::transfer_data(std::move(t)));
		uart0_handler.transmit(std::move(resp));
		HAL_GPIO_TogglePin(DBG1_GPIO_Port, DBG1_Pin);
	});

	//float val = 1;

	/*const float forw = 0.05, forw_fast = 0.25, back = -0.05, right = 0, left = 1, straight = 0.5;
	bool ir_now, ir_last, found = false;
	uint32_t on_time, off_time, time;*/

	//steering.set(straight);

	while (1)
	{
		uart0_interface.main_task();
		uart0_handler.main_task();

		//uart0_handler.main_task();

		//steering.set(val);
		//val = -val;

		//for (float d = -0.3; d <= 0.5; d += 0.1)
		{
			//steering.set(d);
			//drive.set(d > 0.1 ? 0.1 - d : d);
			//HAL_Delay(200);
		}

		/*ir_now = ir_pin.read();
		time = HAL_GetTick();
		Debug_Print("IR: %i\n", ir_now);

		if (ir_now && !ir_last) on_time = time;
		if (!ir_now && ir_last) off_time = time;

		if (!ir_now && off_time > time - 100 && on_time < time - 3000)
		{
			drive.set(0);
			HAL_Delay(1000);
			steering.set(right);
			HAL_Delay(300);
			drive.set(forw_fast);
			HAL_Delay(3000);
			steering.set(left);
			HAL_Delay(3000);
			drive.set(0);
			steering.set(straight);
			found = true;
		}
		else if (!found)
		{
			drive.set(forw);
		}

		ir_last = ir_now;
		HAL_Delay(100);*/

	}
}




