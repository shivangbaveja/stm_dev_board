#include "ppm.h"

// All the variables required for ppm capture and decode
uint32_t channel_pwm[6];
uint8_t ppm_success=0;
uint32_t ppm_channel_index=0, ppm_start_time=0, ppm_end_time=0;

void ppm_config()
{

}

void print_channel_values()
{
	uint8_t i=0,len=0;
	unsigned char str[10];
	for(i=0;i<8;i++)
	{
		UARTSend("Ch",2);
		serial_put_char(i+49);
		UARTSend(":",1);
		len=uint_to_serial(str,channel_pwm[i]);
		UARTSend(str,len);
		UARTSend(",\n\r",3);
	}
}
