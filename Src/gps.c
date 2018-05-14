#include <time.h>
#include <string.h>
#include <stdio.h>

#include "gps.h"

/* this function does not include time accuracy past decimal point (if any) in GPRMC */
unsigned long UtcTimeFromGprmcDateTimeStrings(const char* dateStr, const char* timeStr) {
	struct tm t;
	memset(&t, 0, sizeof(struct tm));
	char str[3];
	str[2] = 0;
	str[0] = dateStr[0];
	str[1] = dateStr[1];
	sscanf(str, "%02d", &t.tm_mday);
	str[0] = dateStr[2];
	str[1] = dateStr[3];
	sscanf(str, "%02d", &t.tm_mon);
	t.tm_mon--; /* month is 0-based */
	
	str[0] = dateStr[4];
	str[1] = dateStr[5];
	sscanf(str, "%02d", &t.tm_year);
	t.tm_year += (2000 - 1900); /* 2017 comes as 17. time base is from 1900 */

	str[0] = timeStr[0];
	str[1] = timeStr[1];
	sscanf(str, "%02d", &t.tm_hour);
	
	str[0] = timeStr[2];
	str[1] = timeStr[3];
	sscanf(str, "%02d", &t.tm_min);
	
	str[0] = timeStr[4];
	str[1] = timeStr[5];
	sscanf(str, "%02d", &t.tm_sec);
	
	time_t seconds = mktime(&t);
	return seconds;
}
