/**
 * @file microros_time.c
 *
 * @brief CAuDri - POSIX time adapters for the bare-metal C library
 */

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

#define NANOSECONDS_PER_SECOND 1000000000ULL
/**
 * @brief Read monotonic time since firmware startup.
 *
 * micro-ROS uses this clock to measure durations and calculate the synchronized
 * agent epoch offset. It must remain independent of ROS connectivity.
 *
 * The firmware has no independent Unix wall clock, so all POSIX clock ids used
 * by linked middleware are mapped to the same monotonic FreeRTOS tick clock.
 * ROS epoch time is provided separately after synchronization with the agent.
 *
 * @param clock_id POSIX clock identifier; ignored on this bare-metal target.
 * @param time Destination for elapsed seconds and nanoseconds.
 * @return 0 on success or -1 with errno set to EINVAL for unsupported input.
 */
int clock_gettime(clockid_t clock_id, struct timespec* time) {
    (void)clock_id;

    if (time == NULL) {
        errno = EINVAL;
        return -1;
    }

    TimeOut_t current_time = {0};
    vTaskSetTimeOutState(&current_time);

    const uint64_t ticks = ((uint64_t)current_time.xOverflowCount << (sizeof(TickType_t) * CHAR_BIT)) + current_time.xTimeOnEntering;
    time->tv_sec = (time_t)(ticks / configTICK_RATE_HZ);
    time->tv_nsec = (long)((ticks % configTICK_RATE_HZ) * NANOSECONDS_PER_SECOND / configTICK_RATE_HZ);
    return 0;
}

/**
 * @brief Report that no Unix wall clock is provided by the firmware.
 *
 * Some linked library code references this syscall even though the firmware
 * does not provide wall-clock time. Defining the stub prevents libnosys from
 * emitting a linker warning while preserving honest wall-clock semantics.
 *
 * @param time_value Unused destination for Unix wall time.
 * @param timezone Unused timezone information.
 * @return Always -1 with errno set to ENOSYS.
 */
int _gettimeofday(struct timeval* time_value, void* timezone) {
    (void)time_value;
    (void)timezone;
    errno = ENOSYS;
    return -1;
}
