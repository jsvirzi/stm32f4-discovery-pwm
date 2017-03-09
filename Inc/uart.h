#ifndef UART_H
#define UART_H

#include "serial.h"

#define uartRxNE 0x20
#define uartTxE 0x80
#define uartRxNEIE 0x20
#define uartTxEIE 0x80

extern SimpleCircularQueue uart1Queue;
extern SimpleCircularQueue uart2Queue;

void initUarts(void);
void procUart(UART_HandleTypeDef *huart);
void uartSendChar(UART_HandleTypeDef *huart, unsigned char ch);

#endif
