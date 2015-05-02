#include "ppm.h"
#include "serial_interface.h"

int main(void)
{
	serial_config();
	ppm_config();
    while(1)
    {
    	if(ppm_success==1)
    	{
    		ppm_success=0;
    		print_channel_values();
    	}
    }
}
