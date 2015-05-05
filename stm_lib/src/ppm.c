#include "ppm.h"

// All the variables required for ppm capture and decode
uint32_t channel_pulses[7];
uint32_t last_pulse_time=0;
uint8_t ppm_current_channel=0, ppm_data_ok=0, ppm_frame_complete=0;

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

void ppm_config()
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enabling clock for TIM3 and GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_DeInit(TIM3);

	/* TIM1 channel 1 pin (PA.06) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period=0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler= (72-1);
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_Cmd(TIM1, ENABLE);

	/* Enable CC1 interrupt */
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);//TIM_IT_CC1

	/* Clear CC1 Flag*/
	TIM_ClearFlag(TIM1, TIM_FLAG_CC1);
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
}

void TIM1_CC_IRQHandler()
{
		print_channel_values();
////	if ((TIM3_SR & PPM_CC_IF) != 0) {
////	    timer_clear_flag(TIM2, PPM_CC_IF);
////
////	    uint32_t now = timer_get_counter(TIM2) + timer_rollover_cnt;
////	    ppm_decode_frame(now);
////	  } else if ((TIM3_SR & TIM_SR_UIF) != 0) {
////	    timer_rollover_cnt += (1 << 16);
////	    timer_clear_flag(TIM2, TIM_SR_UIF);
////	  }
}

void radio_input_init()
{
	last_pulse_time=0;
	ppm_current_channel=RADIO_CHANNEL_NUM;
	ppm_data_ok=0;
	ppm_frame_complete=0;
}


/**
 * Decoding a PPM frame.
 * A valid ppm frame:
 * - Frame space
 * - correct number of channels
 * - Frame space */
void ppm_decode_frame(uint32_t pulse_time)
{
  uint32_t length = pulse_time - last_pulse_time;
  last_pulse_time = pulse_time;

  if (ppm_current_channel == RADIO_CHANNEL_NUM)
  {
	  if (length > PPM_FRAME_SYNC_MIN_LEN &&
        length < PPM_FRAME_SYNC_MAX_LEN)
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
      if (ppm_current_channel == RADIO_CHANNEL_NUM) {
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


void print_channel_values()
{
	uint8_t i=0,len=0;
	unsigned char str[10];
	for(i=0;i<7;i++)
	{
		UARTSend((unsigned char *)"Ch",2);
		serial_put_char(i+49);
		UARTSend((unsigned char *)":",1);
		len=uint_to_serial(str,channel_pulses[i]);
		UARTSend(str,len);
	}
	UARTSend((unsigned char *)",\r",3);
}
