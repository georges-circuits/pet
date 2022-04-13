
#define SP_NO_IOSTREAM

#include "init.h"

#include "../libprotoserial/libprotoserial/protostacks.hpp"

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "syscalls_retarget.h"

#include "hal_wrapper.hpp"
#include "actuators.hpp"
#include "tof_sensor.hpp"
#include "adxl345.hpp"
#include "drive_logic.hpp"
#include "stm_adc.h"
#include "dsp.h"

#define PI 3.14159265358979323846

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
	MX_SPI2_Init();

	RetargetInit(&huart2);
	Debug_Print("Hi\n");

	STM32ADC_Init(&hadc);
	STM32ADC_Start();

	stm32::timer tim2(&htim2, 31, 999);
	stm32::timer tim3(&htim3, 31, 19999);
	stm32::spi spi2(&hspi2);



	auto steering_actuator = manager.new_actuator<servo>(stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1));
	steering_actuator->set_filter(std::make_unique<filter_linear<float>>(0.08));

	auto drive_actuator = manager.new_actuator<simple_stepper>(
			stm32::timer_oc_channel(&tim2, TIM_CHANNEL_1),
			stm32::gpio_inv(STEP_DIR_GPIO_Port, STEP_DIR_Pin, true),
			stm32::gpio_inv(STEP_EN_GPIO_Port, STEP_EN_Pin, true),
			false
	);
	drive_actuator->set_filter(std::make_unique<filter_linear<float>>(0.05));

	move_manager<drive_output> drive([&](drive_output out){
		drive_actuator->set(out.drive);
		steering_actuator->set(out.steering);
	});



	ADXL345 accel(&spi2, stm32::gpio(SPI2_NCS_GPIO_Port, SPI2_NCS_Pin));
	DSP_Biquad_t accel_x_filter, accel_y_filter;
	DSP_Biquad_Init(&accel_x_filter, BIQUAD_LOWPASS, 5.0, 1000, 0.7);
	DSP_Biquad_Init(&accel_y_filter, BIQUAD_LOWPASS, 5.0, 1000, 0.7);

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
				drive_actuator->set((val - 127) / 100.0);
				break;
			case 2_BYTE:
				steering_actuator->set((val - 127) / 100.0);
				break;
			case 3_BYTE:
				autonomous = val;
				drive.cancel_all();
				break;
			}
		}
	});

	float fi = 0, ir = 0;

	DSP_Biquad_t accel_fi_filter;
	DSP_Biquad_Init(&accel_fi_filter, BIQUAD_LOWPASS, 1.0, 1000, 0.7);

	while (1)
	{
		uart.main_task();
		auto ad = accel.read();
		ad.x = DSP_BiquadDF2(&accel_x_filter, -ad.x + 20);
		ad.y = DSP_BiquadDF2(&accel_y_filter, ad.y);

		bool level = false;
		if (fabs(ad.x) + fabs(ad.y) > 15)
		{
			//fi = (fi * (1.0 - filter)) + (filter * (atan2(ad.y, -ad.x) * 180 / PI));
			fi = atan2(ad.y, ad.x) * 180 / PI;
		}
		else
		{
			fi = 0;
			level = true;
		}
		fi = DSP_BiquadDF2(&accel_fi_filter, fi);


		float _ir = 100 - ((STM32ADC_GetReading(STM32ADC_CHANNEL_IR) / STM32ADC_FULLSCALEVOLTAGE) * 100.0);
		ir = _ir;//(ir * (1.0 - filter)) + (filter * _ir);

		static clock::time_point telemetry_last = clock::time_point(clock::duration(0));
		if (telemetry_last + 200ms < clock::now())
		{
			telemetry_last = clock::now();

			/*auto ad = accel.read();
			Debug_Print("%05d %05d\n", ad.x, ad.y);*/

			/*
			auto data = sp::bytes(0, 0, tof::COUNT);
			for (auto & sensor : tof_sensors)
			{
				auto val = sensor.value();
				data.push_back((sp::byte)(val == tof_sensor::invalid_value ? 0 : val / 10));
			}*/

			float voltage = STM32ADC_GetReading(STM32ADC_CHANNEL_BATT) * 6.6;

			sp::bytes d(5);
			d[0] = (sp::byte)(int)(voltage * 10.0);
			d[1] = (sp::byte)(ir);
			d[2] = (sp::byte)ad.x;
			d[3] = (sp::byte)ad.y;
			d[4] = (sp::byte)((int)fi / 10);
			uart.write_noexcept(sp::fragment(2, std::move(d)));
		}

		if (HAL_GetTick() - last_rx > 3000)
		{
			drive.cancel_all();
			drive_actuator->set(0);
			steering_actuator->set(0);
		}
		else if (autonomous > 0)
		{
			bool on_edge = ir < 10;
			static bool backing_down = false;
			drive.update();

			if (level)
			{
				if (drive.is_moving())
					drive.cancel_all();

				backing_down = false;
			}
			else if (on_edge)
			{
				if (!backing_down)
				{
					drive.cancel_all();
					backing_down = true;
					drive.next_move<drive_const>(-0.7, 0, [&, started = clock::now()](){
						if (started + 3s < clock::now())
						{
							backing_down = false;
							return true;
						}
						return false;
					});
				}
			}
			else if (!drive.is_moving())
			{
				const float threshold = 15.0;
				if (fabs(fi) < threshold)
				{
					drive.next_move<drive_const>(0.5, 0, [&](){
						return fabs(fi) > threshold;
					});
				}
				else if (fi > threshold)
				{
					drive.next_move<drive_pivot>(0.7, -0.8, 2s, [&](){
						return fi < threshold / 2.0;
					});
				}
				else if (fi < -threshold)
				{
					drive.next_move<drive_pivot>(0.7, 0.8, 2s, [&](){
						return fi > -threshold / 2.0;
					});
				}
			}
		}
	}
}




