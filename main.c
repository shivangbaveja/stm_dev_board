#include "ppm.h"
#include "serial_interface.h"

int main(void)
{
	int i;
	serial_config();
	ppm_config();
	led_config();
    while(1)
    {
    	//just to check if serial putput is working or not
    	//const unsigned char menu[] = " Welcome to CooCox!\r\n";
    	//UARTSend(menu, sizeof(menu));

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
