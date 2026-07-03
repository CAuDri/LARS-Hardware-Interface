/**
 * @file microros_allocator.c
 *
 * @brief CAuDri - rcl allocator adapter for the shared FreeRTOS heap
 * 
 * This file provides an implementation of the rcl_allocator_t interface using the FreeRTOS memory management functions. It allows micro-ROS to use the FreeRTOS heap for dynamic memory allocation.
 * 
 * A custom heap implementation (custom_heap_4.h) is used to provide realloc and calloc functionality, which are not available in the standard FreeRTOS memory management API.
 */

#include "microros_allocator.h"

#include <rcutils/allocator.h>

#include "FreeRTOS.h"
#include "custom_heap_4.h"

static void *allocate(size_t size, void *state) {
    (void)state;
    return pvPortMalloc(size);
}

static void deallocate(void *pointer, void *state) {
    (void)state;
    vPortFree(pointer);
}

static void *reallocate(void *pointer, size_t size, void *state) {
    (void)state;
    return pvPortRealloc(pointer, size);
}

static void *zero_allocate(size_t number_of_elements, size_t element_size, void *state) {
    (void)state;
    return pvPortCalloc(number_of_elements, element_size);
}

rcl_allocator_t microros_get_allocator(void) {
    rcl_allocator_t allocator = {
        .allocate = allocate,
        .deallocate = deallocate,
        .reallocate = reallocate,
        .zero_allocate = zero_allocate,
        .state = NULL,
    };
    return allocator;
}

rcl_ret_t microros_set_default_allocator(void) {
    rcl_allocator_t allocator = microros_get_allocator();
    return rcutils_set_default_allocator(&allocator) ? RCL_RET_OK : RCL_RET_ERROR;
}
