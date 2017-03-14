/* file serial.h */

#ifndef SERIAL_H
#define SERIAL_H

typedef struct {
	unsigned char *buff;
	int head, tail, length, mask, id;
} SimpleCircularQueue;

void initSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *buff, int length, int id);
void pushSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int nChars);
int popSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int maxChars);
int syncSerialStream(SimpleCircularQueue *queue, const char *header, const char *trailer, int *start, int *stop);
int split(SimpleCircularQueue *queue, int first, int final, char **fields, int maxFields);
void queueSendString(SimpleCircularQueue *queue, char *str, int nChars);

#endif
