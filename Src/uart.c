#include "serial.h"
#include "uart.h"

SimpleCircularQueue uart1Queue;
SimpleCircularQueue uart2Queue;

/* specific to our system */
int initUarts() {
	initSimpleCircularQueue(&uart1Queue, uart1RxBuffer, uart1RxBufferSize);
	initSimpleCircularQueue(&uart2Queue, uart2RxBuffer, uart2RxBufferSize);
}