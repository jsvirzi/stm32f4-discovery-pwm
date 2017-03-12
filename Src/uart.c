/* system files */
#include "usart.h"
#include "string.h"

/* private files */
#include "serial.h"
#include "uart.h"

SimpleCircularQueue uart1RxQueue;
SimpleCircularQueue uart2TxQueue;
SimpleCircularQueue uart2RxQueue;

const int uart1RxBufferSize = 1024;
const int uart1RxBufferSizeMask = 1023;
unsigned char uart1RxBuffer[uart1RxBufferSize];

const int uart2RxBufferSize = 1024;
const int uart2RxBufferSizeMask = 1023;
unsigned char uart2RxBuffer[uart2RxBufferSize];

const int fieldSize = 32;
const int numberOfFields = 16;
char fields[numberOfFields][fieldSize];

const int uart2TxBufferSize = 16 * 1024;
unsigned char uart2TxBuffer[uart2TxBufferSize];

/* specific to our system */
void initUarts() {
	initSimpleCircularQueue(&uart1RxQueue, uart1RxBuffer, uart1RxBufferSize);
	initSimpleCircularQueue(&uart2TxQueue, uart2TxBuffer, uart2TxBufferSize);
	initSimpleCircularQueue(&uart2RxQueue, uart2RxBuffer, uart2RxBufferSize);
}

void procUart(UART_HandleTypeDef *huart, SimpleCircularQueue *queue) {
	if((huart->Instance->SR & uartTxE) == 0) return; /* busy */
	unsigned char ch;
	int nChars = popSimpleCircularQueue(queue, &ch, 1);
	if(nChars == 0) return; /* nothing to do */
	huart->Instance->DR = ch;
}

void processUarts() {
	procUart(&huart2, &uart2TxQueue);
}

/* TODO get rid of this function in current format */
void uartSendChar(UART_HandleTypeDef *huart, unsigned char ch) {
	if(huart->Instance->SR & uartTxE) {
		huart->Instance->DR = ch;
	}
}

int splitString(SimpleCircularQueue *queue, int first, int final) {
	return split(queue, first, final, (char **)fields, numberOfFields);
}

void cat(char *str) {
	queueSendString(&uart2TxQueue, str, 0);
	queueSendString(&uart2TxQueue, "\r", 1);
}
