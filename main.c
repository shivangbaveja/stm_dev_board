#include "ppm.h"
#include "serial_interface.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"

uint32_t time_now=0,old_time=0;

int main(void)
{
	int i;
	serial_config();
	ppm_config();
	led_config();
	radio_input_init();
	timing_config();
    while(1)
    {
    	//As soon as the whole ppm frame is received, ppm_frame_complete is set to 1
    	//All the values are then sent on one UART port using function print_channel_values
		if(ppm_frame_complete==1)
		{
			ppm_frame_complete=0;
			print_channel_values();
		}

    	//LED Blinking code to ensure that the code is working
    	 /* Toggle LEDs which connected to PF6*/
		GPIOF->ODR ^= GPIO_Pin_6;
		/* delay */
		for(i=0;i<0x10000;i++);

		/* Toggle LEDs which connected to PF9*/
		GPIOF->ODR ^= GPIO_Pin_9;
		/* delay */
		for(i=0;i<0x10000;i++);
    }
}
