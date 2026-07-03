/**
 * @file custom_heap_4.h
 *
 * @brief CAuDri - Extensions to the shared FreeRTOS heap_4 allocator
 */
#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *pvPortRealloc(void *pointer, size_t size);
void *pvPortCalloc(size_t number_of_elements, size_t element_size);

#ifdef __cplusplus
}
#endif
