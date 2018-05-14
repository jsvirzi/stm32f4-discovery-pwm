#ifndef UART_H
#define UART_H

#include "serial.h"

#define uartRxNE 0x20
#define uartTxE 0x80
#define uartRxNEIE 0x20
#define uartTxEIE 0x80

extern SimpleCircularQueue uart1TxQueue;
extern SimpleCircularQueue uart1RxQueue;
extern SimpleCircularQueue uart2TxQueue;
extern SimpleCircularQueue uart2RxQueue;

void initUarts(void);
void procUart(UART_HandleTypeDef *huart, SimpleCircularQueue *queue);
void uartSendChar(UART_HandleTypeDef *huart, unsigned char ch);
void processUarts(void);
void cat(char *str);
void display(int line, const char *str, int len);
void displayOn(int flag);
int splitString(SimpleCircularQueue *queue, int first, int final);
char *getField(int id, int fieldIndex);

#endif
