#ifndef serial_INTERFACE_H
#define serial_INTERFACE_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define SERIAL_TX_BUFF_SIZE 		100
#define SERIAL_RX_BUFF_SIZE 		100

extern uint8_t serial_tx_buff[SERIAL_TX_BUFF_SIZE];
extern uint8_t serial_rx_buff[SERIAL_RX_BUFF_SIZE];
extern volatile uint16_t serial_rx_index_wr, serial_rx_index_rd;
extern volatile uint16_t serial_rx_count;
extern volatile uint16_t serial_tx_index_rd,serial_tx_index_wr,serial_tx_count;

void serial_config();
void serial_put_char(uint8_t data);
char serial_get_char();
void clear_serial_RX_data();
int uint_to_serial(unsigned char str[10],uint32_t val);
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);

#endif
