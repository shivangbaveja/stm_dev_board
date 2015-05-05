#include "serial_interface.h"

uint8_t serial_tx_buff[SERIAL_TX_BUFF_SIZE];
uint8_t serial_rx_buff[SERIAL_RX_BUFF_SIZE];
volatile uint16_t serial_rx_index_wr=0, serial_rx_index_rd=0;
volatile uint16_t serial_rx_count=0;
volatile uint16_t serial_tx_index_rd=0,serial_tx_index_wr=0,serial_tx_count=0;

void serial_config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}


void serial_put_char(uint8_t data)
{
	serial_tx_buff[serial_tx_index_wr++]=data;
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	serial_tx_count++;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	if(serial_tx_index_wr == SERIAL_TX_BUFF_SIZE)
		serial_tx_index_wr = 0;
}


char serial_get_char()
{
    char data;
    if(serial_rx_count==0)
    	return 0;
    data = serial_rx_buff[serial_rx_index_rd++];
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    serial_rx_count--;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if(serial_rx_index_rd == SERIAL_RX_BUFF_SIZE)
    	serial_rx_index_rd=0;
    return data;
}

void USART2_IRQHandler(void)
{
	if ((USART2->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
		serial_rx_buff[serial_rx_index_wr++] = (u8) USART_ReceiveData(USART2);
		serial_rx_count++;
		if(serial_rx_index_wr==SERIAL_RX_BUFF_SIZE)
		{
			serial_rx_index_wr = 0;
		}
	}
	if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
	{
		if(serial_tx_count==0)
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

		else
		{
			USART_SendData(USART2, (uint16_t) serial_tx_buff[serial_tx_index_rd++]);
			serial_tx_count--;
			if(serial_tx_index_rd == SERIAL_TX_BUFF_SIZE)
				serial_tx_index_rd = 0;
		}
	}
}

void clear_serial_RX_data()
{
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	serial_rx_count = 0;
	serial_rx_index_wr = 0;
	serial_rx_index_rd = 0;
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

int uint_to_serial(unsigned char str[10],uint32_t val)
{
    int i=0, rem, len = 0;
    uint32_t n;
    n=val;
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (; i < len; i++)
    {
        rem = val % 10;
        val = val / 10;
        str[len - (i+1)] = rem + '0';
    }
    str[len] = '\0';
    return len;
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART2, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
        }
    }
}
