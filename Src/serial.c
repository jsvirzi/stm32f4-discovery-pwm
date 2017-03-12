#include "string.h"
#include "stdio.h"

#include "serial.h"

/* TODO using this in this function for debugging */
void cat(char *str);

/* note: length must be a power of 2 */
void initSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *buff, int length) {
	queue->buff = buff;
	queue->length = length;
	queue->head = 0;
	queue->tail = 0;
	queue->mask = length - 1;
}

void pushSimpleCircularQueue(SimpleCircularQueue *queue, unsigned char *ch, int nChars) {
	if(nChars == 0) {
		nChars = strlen((char *)ch);
	}
	int n = nChars; /* save value */
	while(nChars--) {
		queue->buff[queue->head] = *ch++;
		queue->head = (queue->head + 1) & queue->mask;
	}
}

void queueSendString(SimpleCircularQueue *queue, char *str, int nChars) {
	pushSimpleCircularQueue(queue, (unsigned char *)str, nChars);
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

/* splits csv fields inside queue into different fields */
int split(SimpleCircularQueue *queue, int first, int final, char **fields, int maxFields) {
	char delimiter = ',';
	int i, pos, fieldIndex = 0;
	char *p = fields[fieldIndex];
	i = 0;
	for(pos=first;pos!=final;) {
		char ch = (unsigned char)queue->buff[pos];
		if(ch == delimiter) {
			p[i] = 0; /* finish out current field */
			p = fields[++fieldIndex]; /* next field */
			if(fieldIndex >= maxFields) break;
			i = 0; /* reset position within field */
		} else {
			p[i++] = ch;
		}
		pos = (pos + 1) & queue->mask;
	}
	return fieldIndex;
}

/* functions like strncmp except the source string is a queue. 0 = successful comparison */
int strncmpQueue(SimpleCircularQueue *queue, int start, const char *s, int length) {
	if(length == 0) length = strlen(s);
	char ch;
	for(int i=0;i<length;++i) {
		ch = queue->buff[(start + i) & queue->mask];
		if(ch != s[i]) {
			return ch - s[i];
		}
	}
	return 0;
}

int verbose = 1;
char logBuffer[1024];
/* 0 is success */
int syncSerialStream(SimpleCircularQueue *queue, const char *header, const char *trailer, int *start, int *stop) {
	int i, j, k, lenh = strlen(header), lent = strlen(trailer);
	int head = queue->head, tail = queue->tail;

if(verbose) {
	snprintf(logBuffer, sizeof(logBuffer), "syncSerialStream(): head = %d tail = %d\n", head, tail);
	cat(logBuffer);
}

	int n = head - tail;
	if(n < 0) n += queue->length;
	k = n - lenh - lent;
	if(k < 0) return 0; /* not enough data collected */

if(verbose) {
	snprintf(logBuffer, sizeof(logBuffer), "syncSerialStream(): enough data\n");
}

	for(i=0;i<n;++i) {
		j = tail + i;
		if(strncmpQueue(queue, j, header, lenh)) continue;
		*start = j & queue->mask;
		for(j=i+lenh;j<n;++j) {
			if(strncmpQueue(queue, tail + j, trailer, lent)) continue;
			*stop = (tail + j + lent) & queue->mask;
			
if(verbose) {
	snprintf(logBuffer, sizeof(logBuffer), "syncSerialStream(): start = %d stop = %d buffer = [", *start, *stop);
	int index = strlen(logBuffer);
	j = *stop - *start;
	if(j < 0) j += queue->length;
	for(i=0;i<j;++i) {
		snprintf(logBuffer + index, sizeof(logBuffer) - index, "%c", queue->buff[(*start) + i]);
		++index;
	}
	snprintf(logBuffer + index, sizeof(logBuffer) - index, "]\n");
	cat(logBuffer);
}
			return 0;
		}
	}
	return 1;
}
