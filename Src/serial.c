#include "serial.h"

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

SimpleCircularQueue uart1Queue;
SimpleCircularQueue uart2Queue;

/* note: length must be a power of 2 */
int initSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *buff, int length) {
	queue->buff = buff;
	queue->length = length;
	queue->head = 0;
	queue->tail = 0;
	queue->mask = length - 1;
}

int pushSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int nChars) {
	int n = nChars; /* save value */
	while(nChars--) {
		queue->buff[queue->head] = *ch++;
	}
	queue->head = (queue->head + n) & queue->mask; /* update tail after bytes have been transferred */
}

int popSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int maxChars) {
	int nChars = 0;
	while(queue->head != queue->tail) {
		if(nChars >= maxChars) break;
		++nChars;
		*ch = queue->buff[queue->tail];
		queue->tail = (queue->tail + 1) & queue->mask;
	}
	return nChars;
}

