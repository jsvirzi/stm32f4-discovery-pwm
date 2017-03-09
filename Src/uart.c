/* system files */
#include "usart.h"

/* private files */
#include "serial.h"
#include "uart.h"

SimpleCircularQueue uart1Queue;
SimpleCircularQueue uart2Queue;

const int uart1RxBufferSize = 1024;
const int uart1RxBufferSizeMask = 1023;
unsigned char uart1RxBuffer[uart1RxBufferSize];
int uart1RxBufferHead = 0;
int uart1RxBufferTail = 0;

const int uart2RxBufferSize = 1024;
const int uart2RxBufferSizeMask = 1023;
unsigned char uart2RxBuffer[uart2RxBufferSize];
int uart2RxBufferHead = 0;
int uart2RxBufferTail = 0;

/* specific to our system */
void initUarts() {
	initSimpleCircularQueue(&uart1Queue, uart1RxBuffer, uart1RxBufferSize);
	initSimpleCircularQueue(&uart2Queue, uart2RxBuffer, uart2RxBufferSize);
}

void procUart(UART_HandleTypeDef *huart) {
}

void uartSendChar(UART_HandleTypeDef *huart, unsigned char ch) {
	if(huart->Instance->SR & uartTxE) {
		huart->Instance->DR = ch;
	}
}
