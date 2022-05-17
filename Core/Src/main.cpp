
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
#include "pid_controller.h"

#define PI 3.14159265358979323846

using namespace sp::literals;


static UART_HandleTypeDef *uart0_huart = &huart1;
static sp::uart_interface *uart0_handle;
static tof_sensor_manager *tof_sensors_handle;

static actuator_manager<actuator<float>> manager;

static const float MIC_LOW = 0.017, MIC_HIGH = 0.020;
static const clock::duration MIC_MIN_HIGH_TIME = 100ms;
static DSP_Biquad_t mic_bandpass_filter;
static DSP_MovingAvg_t mic_avg_filter;
static uint mic_detect_count;
static clock::time_point mic_detect_last = clock::time_point{clock::duration{0}};
static float mic_amplitude;


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

	static clock::time_point high_start = clock::time_point{clock::duration{0}};
	static bool triggered = false, incremented = false;
	DSP_MovingAvg_Put(&mic_avg_filter, abs(DSP_BiquadDF2(&mic_bandpass_filter, STM32ADC_GetReading(STM32ADC_CHANNEL_MIC))));
	mic_amplitude = mic_avg_filter.value;

	if (mic_amplitude <= MIC_LOW)
	{
		triggered = false;
		incremented = false;
	}
	else if (!triggered && mic_amplitude >= MIC_HIGH)
	{
		triggered = true;
		high_start = clock::now();
	}
	else if (triggered && high_start + MIC_MIN_HIGH_TIME < clock::now() && !incremented)
	{
		incremented = true;
		++mic_detect_count;
		mic_detect_last = clock::now();
	}
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

	/* this will get reinitialized later once we know the true sample rate */
	DSP_Biquad_Init(&mic_bandpass_filter, BIQUAD_BANDPASS, 440, 3200, 0.7);
	DSP_MovingAvg_Init(&mic_avg_filter, 300, 0);

	STM32ADC_Init(&hadc);
	STM32ADC_Start();

	stm32::timer tim2(&htim2, 31, 999);
	stm32::timer tim3(&htim3, 31, 19999);
	stm32::spi spi2(&hspi2);



	auto steering_actuator = manager.new_actuator<servo>(
			stm32::timer_pwm_channel(&tim3, TIM_CHANNEL_1)
	);
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
	DSP_Biquad_t accel_x_filter, accel_y_filter, accel_fi_filter;
	DSP_Biquad_Init(&accel_x_filter, BIQUAD_LOWPASS, 5.0, 1000, 0.7);
	DSP_Biquad_Init(&accel_y_filter, BIQUAD_LOWPASS, 5.0, 1000, 0.7);
	DSP_Biquad_Init(&accel_fi_filter, BIQUAD_LOWPASS, 1.0, 1000, 0.7);

	PID_t balancer_pid;
	PID_Init(&balancer_pid);
	PID_Set_MinMax(&balancer_pid, -0.8, 0.8);
	PID_Set_I_DivideDecrement(&balancer_pid, 1.01);
	PID_Set_Gains(&balancer_pid, 1.5, 0.0, 0.7);

	tof_sensor_manager tof_sensors;
	tof_sensors_handle = &tof_sensors;
	tof_sensors.start_receive(&huart4);
	tof_sensors.start_receive(&huart5);

	//sp::stack::uart_115200 uart(uart0_huart, 0, 1);
	//uart0_handle = &uart.interface;

	sp::uart_interface uart(uart0_huart, 0, 1, 10, 64, 256);
	uart0_handle = &uart;



	while (!STM32ADC_GetSPS());
	uint32_t adc_sps = STM32ADC_GetSPS();
	Debug_Print("adc sps: %u\n", adc_sps);

	STM32ADC_Stop();
	/*DSP_Biquad_Init(&mic_bandpass_filter, BIQUAD_BANDPASS, 440, adc_sps, 20);
	STM32ADC_Start();*/



	/* motor handler timer */
	HAL_TIM_Base_Start_IT(&htim6);


	/*uart0_interface.receive_event.subscribe([&](sp::fragment f) {
		auto check = sp::footers::crc32(f.data());
		uart0_interface.write(sp::fragment(f.source(), std::move(f.data())));
		HAL_GPIO_TogglePin(DBG1_GPIO_Port, DBG1_Pin);
	});*/

	static uint32_t last_rx = 0;
	int autonomous = 0;
	float fi = 0, ir = 0;
	ADXL345::data ad_raw, ad, ad_cal;

	//uart.transfer_receive_subscribe([&](sp::transfer t) {
	uart.receive_event.subscribe([&](sp::fragment t) {
		last_rx = HAL_GetTick();
		//if (t.data_size() == 2)
		auto data = t.data();
		if (data.size() == 2)
		{
			//auto data = t.data_contiguous();
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
			case 4_BYTE:
				ad_cal = ad_raw;
				break;
			case 5_BYTE:
				balancer_pid.P_gain = (int)val / 10.0;
				break;
			case 6_BYTE:
				balancer_pid.I_gain = (int)val / 10.0;
				break;
			case 7_BYTE:
				balancer_pid.D_gain = (int)val / 10.0;
				break;
			}
		}
	});

	while (1)
	{
		uart.main_task();

		ad_raw = accel.read();
		ad_raw.x = DSP_BiquadDF2(&accel_x_filter, -ad_raw.x);
		ad_raw.y = DSP_BiquadDF2(&accel_y_filter, ad_raw.y);
		ad.x = ad_raw.x - ad_cal.x;
		ad.y = ad_raw.y - ad_cal.y;

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


		ir = 100 - ((STM32ADC_GetReading(STM32ADC_CHANNEL_IR) / STM32ADC_FULLSCALEVOLTAGE) * 100.0);

		static clock::time_point telemetry_last = clock::time_point(clock::duration(0));
		if (telemetry_last + 300ms < clock::now())
		{
			telemetry_last = clock::now();

			Debug_Print("mic %u %i\n", mic_detect_count, (int)(mic_amplitude * 1000));

			/*auto ad = accel.read();
			Debug_Print("%05d %05d\n", ad.x, ad.y);*/

			static uint dist_telem_pres = 0;
			if (++dist_telem_pres >= 3)
			{
				dist_telem_pres = 0;
				auto data = sp::bytes(0, 0, tof::COUNT);
				for (auto & sensor : tof_sensors)
				{
					auto val = sensor.value();
					data.push_back((sp::byte)(val == tof_sensor::invalid_value ? 0 : val / 10));
				}
				uart.write_noexcept(sp::fragment(2, std::move(data)));
			}

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
		else if (autonomous == 1)
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
		else if (autonomous == 2)
		{
			static uint last_count = 0;
			drive.update();
			if (last_count != mic_detect_count && mic_detect_last + 2s < clock::now())
			{
				uint beeps = mic_detect_count - last_count;
				last_count = mic_detect_count;

				drive.cancel_all();
				switch (beeps) {
					case 1:
						drive.next_move<drive_pivot>(0.7, 0.8, 2s, [started = clock::now()](){
							return started + 10s < clock::now();
						});
						break;
					case 2:
						drive.next_move<drive_pivot>(0.7, -0.8, 2s, [started = clock::now()](){
							return started + 10s < clock::now();
						});
						break;
					case 3:
						drive.next_move<drive_const>(0.7, 0, [started = clock::now()](){
							return started + 10s < clock::now();
						});
						break;
					default:
						break;
				}
			}
		}
		else if (autonomous == 3)
		{
			const int16_t max_defl = 150;
			static clock::time_point last = clock::time_point(clock::duration(0));
			if (last + 100ms < clock::now())
			{
				last = clock::now();
				if (abs(ad.x) < max_defl && abs(ad.y) < max_defl)
				{
					auto v = PID_Update_Setpoint_TimeDelta(&balancer_pid, 0, ad.x / 100.0, 1.0);
					drive_actuator->set(-v);
				}
				else
				{
					drive_actuator->set(0);
				}
			}
		}
	}
}




