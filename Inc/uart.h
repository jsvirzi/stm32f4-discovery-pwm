#ifndef UART_H
#define UART_H

#include "serial.h"

#define uartRxNE 0x20
#define uartTxEIE 0x80
#define uartRxNEIE 0x20

extern SimpleCircularQueue uart1Queue;
extern SimpleCircularQueue uart2Queue;

int initUarts();

#endif