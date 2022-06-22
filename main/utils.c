#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "utils.h"


uint32_t utils_time_diff(uint32_t time1, uint32_t time0)
{
	uint32_t diff;
	if(time1 >= time0)
		diff = time1 - time0;
	else
		diff = ((uint32_t)-1) - time0 + time1;
	return diff;
}

uint32_t utils_time_ms_get(void)
{
	struct timespec curr_time;
	uint32_t ms;
	clock_gettime(CLOCK_MONOTONIC, &curr_time);
	ms = curr_time.tv_sec * 1000 + curr_time.tv_nsec / 1000000;
	return ms;
}

uint32_t utils_time_sec_get(void)
{
	struct timespec curr_time;
	clock_gettime(CLOCK_MONOTONIC, &curr_time);
	return curr_time.tv_sec;
}

