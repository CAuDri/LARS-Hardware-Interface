/**
 * @file microros_allocator.h
 *
 * @brief CAuDri - rcl allocator adapter for the shared FreeRTOS heap
 */
#pragma once

#include <rcl/allocator.h>
#include <rcl/types.h>

#ifdef __cplusplus
extern "C" {
#endif

rcl_allocator_t microros_get_allocator(void);
rcl_ret_t microros_set_default_allocator(void);

#ifdef __cplusplus
}
#endif
