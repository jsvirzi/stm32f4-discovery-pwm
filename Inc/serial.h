/* file serial.h */

#ifndef SERIAL_H
#define SERIAL_H

typedef struct {
	unsigned char *buff;
	int head, tail, length, mask;
} SimpleCircularQueue;

void initSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *buff, int length);
void pushSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int nChars);
int popSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int maxChars);

#endif
