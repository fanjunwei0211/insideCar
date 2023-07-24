#ifndef UART_COM_H
#define UART_COM_H
#include "struct_typedef.h"
#include "cmsis_os.h"
#include "usart.h"

extern uint8_t uart6_data;
extern uint8_t uart6_rxbuf[];
extern uint8_t uart6_txbuf[];

void usart_start_it(void);
void Uart6_Receive_Data(uint8_t data);

#endif
