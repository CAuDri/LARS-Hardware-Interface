/**
 * @file microros_time.c
 *
 * @brief CAuDri - POSIX clock adapter backed by the FreeRTOS tick counter
 */

#include <limits.h>
#include <stdint.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#define NANOSECONDS_PER_SECOND 1000000000ULL

int clock_gettime(clockid_t clock_id, struct timespec* time) {
    (void)clock_id;
    if (time == NULL) {
        return -1;
    }

    TimeOut_t current_time = {0};
    vTaskSetTimeOutState(&current_time);

    const uint64_t ticks = ((uint64_t)current_time.xOverflowCount << (sizeof(TickType_t) * CHAR_BIT)) + current_time.xTimeOnEntering;
    time->tv_sec = (time_t)(ticks / configTICK_RATE_HZ);
    time->tv_nsec = (long)((ticks % configTICK_RATE_HZ) * NANOSECONDS_PER_SECOND / configTICK_RATE_HZ);
    return 0;
}
