/* system files */
#include "usart.h"
#include "string.h"

/* private files */
#include "serial.h"
#include "uart.h"

SimpleCircularQueue uart1TxQueue;
SimpleCircularQueue uart1RxQueue;
SimpleCircularQueue uart2TxQueue;
SimpleCircularQueue uart2RxQueue;
SimpleCircularQueue uart6TxQueue;
SimpleCircularQueue uart6RxQueue;

const int uart1RxBufferSize = 1024;
const int uart1RxBufferSizeMask = 1023;
unsigned char uart1RxBuffer[uart1RxBufferSize];

const int uart2RxBufferSize = 1024;
const int uart2RxBufferSizeMask = 1023;
unsigned char uart2RxBuffer[uart2RxBufferSize];

const int uart6RxBufferSize = 128;
const int uart6RxBufferSizeMask = 127;
unsigned char uart6RxBuffer[uart6RxBufferSize];

const int fieldSize = 32;
const int numberOfFields = 16;
char uart1FieldBuffers[numberOfFields][fieldSize];
char uart2FieldBuffers[numberOfFields][fieldSize];
char *uart1Fields[numberOfFields];
char *uart2Fields[numberOfFields];

const int uart1TxBufferSize = 2 * 1024;
unsigned char uart1TxBuffer[uart1TxBufferSize];

const int uart2TxBufferSize = 16 * 1024;
unsigned char uart2TxBuffer[uart2TxBufferSize];

const int uart6TxBufferSize = 256;
unsigned char uart6TxBuffer[uart6TxBufferSize];

char *getField(int id, int fieldIndex) {
	if(id == 1) {
		return uart1Fields[fieldIndex];
	} else if(id == 2) {
		return uart2Fields[fieldIndex];
	}
	return 0;
}

/* specific to our system */
void initUarts() {
	initSimpleCircularQueue(&uart1TxQueue, uart1TxBuffer, uart1TxBufferSize, 1);
	initSimpleCircularQueue(&uart1RxQueue, uart1RxBuffer, uart1RxBufferSize, 1);
	initSimpleCircularQueue(&uart2TxQueue, uart2TxBuffer, uart2TxBufferSize, 2);
	initSimpleCircularQueue(&uart2RxQueue, uart2RxBuffer, uart2RxBufferSize, 2);
	initSimpleCircularQueue(&uart6TxQueue, uart6TxBuffer, uart6TxBufferSize, 6);
	initSimpleCircularQueue(&uart6RxQueue, uart6RxBuffer, uart6RxBufferSize, 6);
	int i;
	for(i=0;i<numberOfFields;++i) {
		uart1Fields[i] = uart1FieldBuffers[i];
		uart2Fields[i] = uart2FieldBuffers[i];
	}
}

void procUart(UART_HandleTypeDef *huart, SimpleCircularQueue *queue) {
	if((huart->Instance->SR & uartTxE) == 0) return; /* busy */
	unsigned char ch;
	int nChars = popSimpleCircularQueue(queue, &ch, 1);
	if(nChars == 0) return; /* nothing to do */
	huart->Instance->DR = ch;
}

void processUarts() {
	procUart(&huart1, &uart1TxQueue);
	procUart(&huart2, &uart2TxQueue);
	procUart(&huart6, &uart6TxQueue);
}

/* TODO get rid of this function in current format */
void uartSendChar(UART_HandleTypeDef *huart, unsigned char ch) {
	if(huart->Instance->SR & uartTxE) {
		huart->Instance->DR = ch;
	}
}

int splitString(SimpleCircularQueue *queue, int first, int final) {
	if(queue->id == 1) {
		return split(queue, first, final, (char **)uart1Fields, numberOfFields);
	} else if(queue->id == 2) {
		return split(queue, first, final, (char **)uart2Fields, numberOfFields);
	}
	return -1;
}

void cat(char *str) {
	queueSendString(&uart2TxQueue, str, 0);
	queueSendString(&uart2TxQueue, "\r", 1);
}

void displayOn(int flag) {
	// char ch[2];
	// if (flag) { ch[0] = 22; ch[1] = 17; }
	// else { ch[0] = 18; ch[1] = 21; }
	// queueSendString(&uart6TxQueue, ch, 2);
}

void display(int line, const char *msg, int len) {
	char str[23]; /* max length */
	if (len == 0) len = strlen(msg);
	if (len > sizeof(str)) { return; }
	int j = 0;
	str[j++] = 0x11;
	str[j++] = 0;
	str[j++] = line;
	for (int i = 0; i < len; ++i) { str[j++] = msg[i]; }
	queueSendString(&uart6TxQueue, str, j);
}
