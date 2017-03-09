#include "serial.h"

/* note: length must be a power of 2 */
void initSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *buff, int length) {
	queue->buff = buff;
	queue->length = length;
	queue->head = 0;
	queue->tail = 0;
	queue->mask = length - 1;
}

void pushSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int nChars) {
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

