/**
  ******************************************************************************
  * @file    TIM/InputCapture/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "serial_interface.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_Input_Capture
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

// Variable definitions
uint8_t CaptureNumber=0;
uint32_t IC3ReadValue1=0, IC3ReadValue2=0, Capture=0;
uint16_t TIM3Freq=0;
uint8_t i=0,len=0;
unsigned char str[10];
uint8_t start=0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */

  /* System Clocks Configuration */
  RCC_Configuration();

  /* NVIC configuration */
  NVIC_Configuration();

  /* Configure the GPIO ports */
  GPIO_Configuration();
  serial_config();

  /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH2 pin (PA.07)
     The Rising edge is used as active edge,
     The TIM3 CCR2 is used to compute the frequency value
  ------------------------------------------------------------ */

  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_Period = 0xffff;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  /* TIM enable counter */
  TIM_Cmd(TIM3, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

  while (1);
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

/**
  * @brief  Configure the GPIOD Pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 channel 2 pin (PA.07) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	 if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET && start==0)
	  {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		 IC3ReadValue1 = TIM_GetCapture2(TIM3);
		 start=1;
		 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		 TIM_ICInit(TIM3, &TIM_ICInitStructure);
	  }
	 else
	 {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		 IC3ReadValue2 = TIM_GetCapture2(TIM3);
		 if (IC3ReadValue2 > IC3ReadValue1)
			{
			Capture = (IC3ReadValue2 - IC3ReadValue1);
			}
			else
			{
			Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2);
			}
		 start=0;
		 TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		 TIM_ICInit(TIM3, &TIM_ICInitStructure);
		 UARTSend((unsigned char *)"Ch:",3);
			      len=uint_to_serial(str,Capture);
			      UARTSend(str,len);
			      UARTSend((unsigned char *)",\r",3);
	 }
//  if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
//  {
//    /* Clear TIM3 Capture compare interrupt pending bit */
//    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//    if(CaptureNumber == 0)
//    {
//      /* Get the Input Capture value */
//      IC3ReadValue1 = TIM_GetCapture2(TIM3);
//      CaptureNumber = 1;
//    }
//    else if(CaptureNumber == 1)
//    {
//      /* Get the Input Capture value */
//      IC3ReadValue2 = TIM_GetCapture2(TIM3);
//
//      /* Capture computation */
//      if (IC3ReadValue2 > IC3ReadValue1)
//      {
//        Capture = (IC3ReadValue2 - IC3ReadValue1);
//      }
//      else
//      {
//        Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2);
//      }
//      /* Frequency computation */
//      TIM3Freq = (uint32_t)1612903 / Capture;
//      CaptureNumber = 0;
//      UARTSend((unsigned char *)"Ch:",3);
//      len=uint_to_serial(str,TIM3Freq);
//      UARTSend(str,len);
//      UARTSend((unsigned char *)",\r",3);
//    }
//  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
