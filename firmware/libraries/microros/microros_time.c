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
#define MICROROS_CLOCK_MONOTONIC ((clockid_t)0)

/**
 * @brief Read monotonic time since firmware startup.
 *
 * micro-ROS uses this clock to measure durations and calculate the synchronized
 * agent epoch offset. It must remain independent of ROS connectivity.
 *
 * Only the monotonic clock identifier used to compile the micro-ROS static
 * library is supported because the firmware has no independent wall clock.
 * ROS epoch time is provided separately by ros::Client after it synchronizes
 * with the agent.
 *
 * @param clock_id POSIX clock identifier; must be the micro-ROS monotonic ID.
 * @param time Destination for elapsed seconds and nanoseconds.
 * @return 0 on success or -1 with errno set to EINVAL for unsupported input.
 */
int clock_gettime(clockid_t clock_id, struct timespec* time) {
    if (clock_id != MICROROS_CLOCK_MONOTONIC || time == NULL) {
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
 * Newlib's time() implementation references this syscall through currently
 * unused rclc action-client code. Defining the stub prevents libnosys from
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
