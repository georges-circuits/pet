

#include "init.h"
#include "../libprotoserial/libprotoserial/interface.hpp"


int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();

	HAL_TIM_Base_Start_IT(&htim2);


	sp::loopback_interface interface(0, 1, 10, 64, 256);

	while (1)
	{

	}
}

