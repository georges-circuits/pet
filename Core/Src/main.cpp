
#define SP_NO_IOSTREAM

#include "init.h"

#include "../libprotoserial/libprotoserial/protostacks.hpp"
#include "../libprotoserial/libprotoserial/clock.hpp"


#include <stdio.h>
#include <stdarg.h>
#include "syscalls_retarget.h"

#include "hal_wrapper.hpp"
#include "actuators.hpp"
#include "tof_sensor.hpp"

#include "stm_adc.h"

using namespace sp::literals;


UART_HandleTypeDef *uart0_huart = &huart1;
sp::uart_interface *uart0_handle;
tof_sensor_manager *tof_sensors_handle;

actuator_manager<actuator<float>> manager;


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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
		manager.update();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	STM32ADC_ConvCpltCallback(hadc);
	STM32ADC_Start(); //FIXME should not be needed
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
	MX_DMA_Init();
	MX_ADC_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();

	//RetargetInit(&huart2);

	//Debug_Print("Hi\n");

	STM32ADC_Init(&hadc);
	STM32ADC_Start();

	stm32::timer tim2(&htim2, 31, 999);
	stm32::timer tim3(&htim3, 31, 19999);

	auto steering = manager.new_actuator<servo>(stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1));
	steering->set_filter(std::make_unique<filter_linear<float>>(0.08));

	auto drive = manager.new_actuator<simple_stepper>(
			stm32::timer_oc_channel(&tim2, TIM_CHANNEL_1),
			stm32::gpio_inv(STEP_DIR_GPIO_Port, STEP_DIR_Pin, true),
			stm32::gpio_inv(STEP_EN_GPIO_Port, STEP_EN_Pin, true),
			false
	);
	drive->set_filter(std::make_unique<filter_linear<float>>(0.05));

	//servo steering(stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1));
	/*simple_stepper drive(
			stm32::timer_oc_channel(&tim2, TIM_CHANNEL_1),
			stm32::gpio_inv(STEP_DIR_GPIO_Port, STEP_DIR_Pin, true),
			stm32::gpio_inv(STEP_EN_GPIO_Port, STEP_EN_Pin, true),
			false
	);*/

	stm32::gpio_inv ir_pin(IR_GPIO_Port, IR_Pin);

	tof_sensor_manager tof_sensors;
	tof_sensors_handle = &tof_sensors;
	tof_sensors.start_receive(&huart4);
	tof_sensors.start_receive(&huart5);

	//sp::stack::uart_115200 uart(uart0_huart, 0, 1);
	//uart0_handle = &uart.interface;

	sp::uart_interface uart(uart0_huart, 0, 1, 10, 64, 256);
	uart0_handle = &uart;

	HAL_TIM_Base_Start_IT(&htim6);

	/*uart0_interface.receive_event.subscribe([&](sp::fragment f) {
		auto check = sp::footers::crc32(f.data());
		uart0_interface.write(sp::fragment(f.source(), std::move(f.data())));
		HAL_GPIO_TogglePin(DBG1_GPIO_Port, DBG1_Pin);
	});*/

	static uint32_t last_rx = 0;
	int autonomous = 0;
	//uart.transfer_receive_subscribe([&](sp::transfer t) {
	uart.receive_event.subscribe([&](sp::fragment t) {

		last_rx = HAL_GetTick();
		//if (t.data_size() == 2)
		if (t.data().size() == 2)
		{
			//auto data = t.data_contiguous();
			auto data = t.data();
			int val = (int)data[1];
			switch (data[0])
			{
			case 1_BYTE:
				drive->set((val - 127) / 100.0);
				break;
			case 2_BYTE:
				steering->set((val - 127) / 100.0);
				break;
			case 3_BYTE:
				autonomous = val;
				break;
			}
		}

	});

	//float val = 1;

	/*const float forw = 0.05, forw_fast = 0.25, back = -0.05, right = 0, left = 1, straight = 0.5;
	bool ir_now, ir_last, found = false;
	uint32_t on_time, off_time, time;*/

	//steering->set(straight);

	while (1)
	{
		uart.main_task();

		static uint32_t tick = 0;
		if (tick + 200 < HAL_GetTick())
		{
			tick = HAL_GetTick();

			/*auto tr = sp::transfer(uart.interface_id());
			tr.set_destination(2);

			auto data = sp::bytes(0, 0, tof::COUNT);
			for (auto & sensor : tof_sensors)
			{
				auto val = sensor.value();
				data.push_back((sp::byte)(val == tof_sensor::invalid_value ? 0 : val / 10));
			}

			tr.push_back(std::move(data));
			uart.transfer_transmit(std::move(tr));*/

			/*HAL_ADC_Start(&hadc);
			HAL_ADC_PollForConversion(&hadc, 5);
			auto val = HAL_ADC_GetValue(&hadc);*/

			float voltage = STM32ADC_GetReading(STM32ADC_CHANNEL_BATT) * 6.6;
			float ir = (STM32ADC_GetReading(STM32ADC_CHANNEL_IR) / STM32ADC_FULLSCALEVOLTAGE) * 255;

			sp::bytes d(2);
			d[0] = (sp::byte)(int)(voltage * 10.0);
			d[1] = (sp::byte)(ir);
			uart.write_noexcept(sp::fragment(2, std::move(d)));
		}

		if (HAL_GetTick() - last_rx > 3000)
		{
			drive->set(0);
			steering->set(0);
		}
		else
		{
			static uint32_t state_changed = 0, next_in = 0, turns = 0;
			const float speed = 0.70, rate = 0.8;
			const uint16_t min_dist = 300;
			if (autonomous > 0)
			{
				switch (autonomous) {
					case 1: /* init */
						drive->set(0);
						steering->set(0);
						next_in = 1;
						turns = 2;
						break;
					case 2: /* creep forward */
						drive->set(speed);
						next_in = 1;
						break;
					case 3: /* detect an obstacle */
						if (tof_sensors.at(tof::S5).value() < min_dist ||
								tof_sensors.at(tof::S6).value() < min_dist)
							next_in = 100;
						else
							next_in = 0;
						break;
					case 4: /* start reversing and turning */
						if (!next_in)
						{
							drive->set(-speed);
							steering->set(rate);
							next_in = 3000;
						}
						break;
					case 5: /* go forward and turn the other way completing the rotation maneuver */
						if (!next_in)
						{
							drive->set(speed);
							steering->set(-rate);
							next_in = 3000;
						}
						break;
					case 6: /* repeat the above turns-times */
						if (turns > 0)
						{
							turns -= 1;
							state_changed = HAL_GetTick();
							autonomous = 4;
						}
						else
							next_in = 1;
						break;
					case 7: /* go forward again */
						if (!next_in)
						{
							drive->set(speed);
							steering->set(0);
							next_in = 10000;
						}
						break;
					default:
						drive->set(0);
						steering->set(0);
						autonomous = 0;
						next_in = 0;
						break;
				}

				if (next_in && HAL_GetTick() - state_changed >= next_in)
				{
					state_changed = HAL_GetTick();
					autonomous += 1;
					next_in = 0;
				}
			}
		}


	}
}




