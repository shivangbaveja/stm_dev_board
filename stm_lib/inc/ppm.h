#ifndef PPM_H
#define PPM_H

#include "serial_interface.h"

// All the variables required for ppm capture and decode
extern uint32_t channel_pwm[6];
extern uint8_t ppm_success;
extern uint32_t ppm_channel_index, ppm_start_time, ppm_end_time;

//function definitions specific to ppm configuration and channel width capture
void ppm_config();		//configure all the hardware interfaces required to get ppm decoded
void print_channel_values();		//write all the ppm channel values on serial interface


















#endif
