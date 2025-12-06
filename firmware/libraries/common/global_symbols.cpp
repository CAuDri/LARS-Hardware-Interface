/**
 * @file global_symbols.cpp
 *
 * @brief CAuDri - Global definitions for required symbols
 *
 * This file provides definitions for global symbols that are required by the linker or runtime environment.
 *
 */
#include "logger.h"
#include "main.h"

// This is necessary for OpenOCD to show the correct FreeRTOS task list
// The variable must be declared globally and volatile to prevent optimization
const volatile UBaseType_t uxTopUsedPriority = configMAX_PRIORITIES - 1;

// Override new and delete operators to use FreeRTOS heap functions
void* operator new(size_t size) {
    LogWarning("System heap allocation detected using the 'new' operator. Will redirect to pvPortMalloc.");
    LogWarning("Consider using static or stack allocation to avoid fragmentation.");
    return pvPortMalloc(size);
}

void operator delete(void* ptr) noexcept { vPortFree(ptr); }

// Override malloc and free to use FreeRTOS heap functions
extern "C" void* malloc(size_t size) {
    LogWarning("System heap allocation detected using 'malloc'. Will redirect to pvPortMalloc.");
    LogWarning("Consider using static or stack allocation to avoid fragmentation.");
    return pvPortMalloc(size);
}

extern "C" void free(void* ptr) { vPortFree(ptr); }