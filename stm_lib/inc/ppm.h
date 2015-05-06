#ifndef PPM_H
#define PPM_H

#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "serial_interface.h"

// Macros specific to the Radio transmitter being used-- Futaba T7C in our case
// All the pulses are in usec
#define PPM_FRAME_SYNC_MIN_LEN  5500		//6202
#define PPM_FRAME_SYNC_MAX_LEN 	15000		//13784
#define PPM_FRAME_DATA_MIN_LEN	800
#define PPM_FRAME_DATA_MAX_LEN	2200
#define RADIO_CHANNEL_NUM	7

// All the variables required for ppm capture and decode
extern uint32_t channel_pulses[7];
extern uint32_t debug_counter;
extern uint8_t ppm_current_channel,ppm_data_ok, ppm_frame_complete;
extern uint32_t IC3ReadValue1, IC3ReadValue2, Capture;
extern uint8_t i,len;
extern unsigned char str[10];
TIM_ICInitTypeDef  TIM_ICInitStructure;
extern uint16_t last, now;

//function definitions specific to ppm configuration and channel width capture
void ppm_config();		//configure all the hardware interfaces required to get ppm decoded
void print_channel_values();		//write all the ppm channel values on serial interface
void ppm_decode_frame();
void radio_input_init();		//Initialize a few variables for ppm capture start from radio
void led_config();
void timing_config();


#endif
