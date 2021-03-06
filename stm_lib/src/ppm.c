/*********************   PPM INPUT LIBRARY *******************
This library is written to take input from Futaba transmitter's ppm output
The output ppm pulses are received, parsed, decoded and then sent to
a serial port.
**************************************************************/
#include "ppm.h"

// All the variables required for ppm capture and decode
uint32_t channel_pulses[7];							//Stores all the instantaneous pulse values
uint32_t debug_counter=0;							//variable for debugging purposes
uint8_t ppm_current_channel=0;						//current channel whose pulse is being currently received
uint8_t ppm_data_ok=0, ppm_frame_complete=0;
uint32_t IC3ReadValue1=0, IC3ReadValue2=0, Capture=0;
uint8_t i=0,len=0;
unsigned char str[10];
uint16_t last=0, now=0;

TIM_ICInitTypeDef  TIM_ICInitStructure;

// Configuring TIM2 for timing purposes, It was only used for debugging purposes
void timing_config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 3600-1;
	TIM_TimeBaseStructure.TIM_Period = 50000;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
}

//Configuring the GPIO pins for blinking the LEDs, indicates if the code is running or not
void led_config()
{
	/* Initialize Leds mounted on STM32 board */
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Initialize LED which connected to PC6,9, Enable the Clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

// Configuring all the interfaces required for getting the PPM input
// All the peripherals that were configured were GPIO A7 (pin for ppm pulse input)
// Channel2 of TIM3 configured as input capture
// The resolution of the timer was such set in a such a way as to get 1us accuracy
void ppm_config()
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitTypeDef GPIO_InitStructure;
	/* TIM3 channel 2 pin (PA.07) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
}

// TIM3 interrupt handler
// captures the counter at interrupt(both rise and fall) and calls ppm_decode_frame function
// to decode the individual channel pulse values
void TIM3_IRQHandler(void)
{
	 if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
	  {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		 now=TIM_GetCapture2(TIM3);
		 ppm_decode_frame();
//		 debug_counter++;
	  }
}

//Called initially to setup a few variables, This is where we can modify code for
// different transmitter types if we want
void radio_input_init()
{
	last=0;
	ppm_current_channel=RADIO_CHANNEL_NUM;
	ppm_data_ok=0;
	ppm_frame_complete=0;
}


/**
 * Decoding a PPM frame.
 * A valid ppm frame:
 * - Waits for the Frame space
 * - gets correct number of channels
 * - sets ppm_frame_complete to 1 for us to use the ppm frame values */
void ppm_decode_frame()
{
	uint16_t length;
	if(now < last)
	{
		length = now + 0xFFFF - last;
		last=now;
	}
	else
	{
		length=now-last;
		last=now;
	}
	if (ppm_current_channel == RADIO_CHANNEL_NUM)
	{
		if (length > PPM_FRAME_SYNC_MIN_LEN &&  length < PPM_FRAME_SYNC_MAX_LEN)
		{
			if(ppm_data_ok)
			{
				ppm_frame_complete = 1;
				ppm_data_ok = 0;
			}
			ppm_current_channel = 0;
		}
		else
		{
			ppm_data_ok = 0;
		}
	}
	else
	{
		if (length > PPM_FRAME_DATA_MIN_LEN && length < PPM_FRAME_DATA_MAX_LEN)
		{
			channel_pulses[ppm_current_channel] = length;
			ppm_current_channel++;
			if (ppm_current_channel == RADIO_CHANNEL_NUM)
			{
				ppm_data_ok = 1;
			}
		}
		else
		{
			ppm_current_channel = RADIO_CHANNEL_NUM;
			ppm_data_ok=0;
		}
	}
}

// Print all the channel's pulse width values and change to next line
void print_channel_values()
{
	uint8_t i=0,len=0;
	unsigned char str[10];
	for(i=0;i<7;i++)
	{
		UARTSend((unsigned char *)",Ch",3);
		serial_put_char(i+49);
		UARTSend((unsigned char *)":",1);
		len=uint_to_serial(str,channel_pulses[i]);
		UARTSend(str,len);
	}
	UARTSend((unsigned char *)"\r",3);
}
