/**
 * @file custom_heap_4.c
 *
 * @brief CAuDri - Shared FreeRTOS heap_4 allocator with realloc and calloc support
 *
 * FreeRTOS and micro-ROS use this single heap through the canonical
 * pvPortMalloc() and vPortFree() API.
 */

#include <stdint.h>
#include <string.h>

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "FreeRTOS.h"
#include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "custom_heap_4.h"

#if (configSUPPORT_DYNAMIC_ALLOCATION == 0)
#error custom_heap_4.c requires configSUPPORT_DYNAMIC_ALLOCATION
#endif

#define heapBITS_PER_BYTE ((size_t)8)
#define heapMINIMUM_BLOCK_SIZE ((size_t)(heap_struct_size << 1U))

typedef struct BlockLink {
    struct BlockLink *next;
    size_t size;
} BlockLink_t;

static uint8_t ucHeap[configTOTAL_HEAP_SIZE];
static BlockLink_t start;
static BlockLink_t *end = NULL;
static size_t free_bytes = 0U;
static size_t minimum_free_bytes = 0U;
static size_t successful_allocations = 0U;
static size_t successful_frees = 0U;
static size_t allocated_bit = 0U;
static const size_t heap_struct_size =
    (sizeof(BlockLink_t) + (portBYTE_ALIGNMENT - 1U)) & ~(size_t)portBYTE_ALIGNMENT_MASK;

static void initialize_heap(void);
static void insert_free_block(BlockLink_t *block);

void *pvPortMalloc(size_t wanted_size) {
    BlockLink_t *block;
    BlockLink_t *previous;
    void *result = NULL;

    vTaskSuspendAll();
    {
        if (end == NULL) {
            initialize_heap();
        }

        if ((wanted_size & allocated_bit) == 0U && wanted_size > 0U &&
            wanted_size <= SIZE_MAX - heap_struct_size) {
            wanted_size += heap_struct_size;
            if ((wanted_size & portBYTE_ALIGNMENT_MASK) != 0U) {
                const size_t padding = portBYTE_ALIGNMENT - (wanted_size & portBYTE_ALIGNMENT_MASK);
                if (wanted_size <= SIZE_MAX - padding) {
                    wanted_size += padding;
                } else {
                    wanted_size = 0U;
                }
            }

            if (wanted_size > 0U && wanted_size <= free_bytes) {
                previous = &start;
                block = start.next;
                while (block->size < wanted_size && block->next != NULL) {
                    previous = block;
                    block = block->next;
                }

                if (block != end) {
                    result = (uint8_t *)block + heap_struct_size;
                    previous->next = block->next;

                    if (block->size - wanted_size > heapMINIMUM_BLOCK_SIZE) {
                        BlockLink_t *remainder = (BlockLink_t *)((uint8_t *)block + wanted_size);
                        remainder->size = block->size - wanted_size;
                        block->size = wanted_size;
                        insert_free_block(remainder);
                    }

                    free_bytes -= block->size;
                    if (free_bytes < minimum_free_bytes) {
                        minimum_free_bytes = free_bytes;
                    }
                    block->size |= allocated_bit;
                    block->next = NULL;
                    successful_allocations++;
                }
            }
        }
        traceMALLOC(result, wanted_size);
    }
    (void)xTaskResumeAll();

#if (configUSE_MALLOC_FAILED_HOOK == 1)
    if (result == NULL) {
        extern void vApplicationMallocFailedHook(void);
        vApplicationMallocFailedHook();
    }
#endif

    configASSERT(result == NULL || (((size_t)result & portBYTE_ALIGNMENT_MASK) == 0U));
    return result;
}

void vPortFree(void *pointer) {
    if (pointer == NULL) {
        return;
    }

    BlockLink_t *block = (BlockLink_t *)((uint8_t *)pointer - heap_struct_size);
    configASSERT((block->size & allocated_bit) != 0U);
    configASSERT(block->next == NULL);

    if ((block->size & allocated_bit) != 0U && block->next == NULL) {
        block->size &= ~allocated_bit;
        vTaskSuspendAll();
        {
            free_bytes += block->size;
            traceFREE(pointer, block->size);
            insert_free_block(block);
            successful_frees++;
        }
        (void)xTaskResumeAll();
    }
}

void *pvPortRealloc(void *pointer, size_t size) {
    if (pointer == NULL) {
        return pvPortMalloc(size);
    }
    if (size == 0U) {
        vPortFree(pointer);
        return NULL;
    }

    const BlockLink_t *old_block = (const BlockLink_t *)((const uint8_t *)pointer - heap_struct_size);
    const size_t old_size = (old_block->size & ~allocated_bit) - heap_struct_size;
    void *replacement = pvPortMalloc(size);
    if (replacement == NULL) {
        return NULL;
    }

    memcpy(replacement, pointer, size < old_size ? size : old_size);
    vPortFree(pointer);
    return replacement;
}

void *pvPortCalloc(size_t number_of_elements, size_t element_size) {
    if (element_size != 0U && number_of_elements > SIZE_MAX / element_size) {
        return NULL;
    }
    const size_t size = number_of_elements * element_size;
    void *pointer = pvPortMalloc(size);
    if (pointer != NULL) {
        memset(pointer, 0, size);
    }
    return pointer;
}

size_t xPortGetFreeHeapSize(void) { return free_bytes; }

size_t xPortGetMinimumEverFreeHeapSize(void) { return minimum_free_bytes; }

void vPortInitialiseBlocks(void) {}

static void initialize_heap(void) {
    size_t address = (size_t)ucHeap;
    size_t total_size = configTOTAL_HEAP_SIZE;
    if ((address & portBYTE_ALIGNMENT_MASK) != 0U) {
        const size_t offset = portBYTE_ALIGNMENT - (address & portBYTE_ALIGNMENT_MASK);
        address += offset;
        total_size -= offset;
    }

    uint8_t *aligned_heap = (uint8_t *)address;
    address = ((size_t)aligned_heap + total_size - heap_struct_size) & ~(size_t)portBYTE_ALIGNMENT_MASK;
    end = (BlockLink_t *)address;
    end->size = 0U;
    end->next = NULL;

    start.size = 0U;
    start.next = (BlockLink_t *)aligned_heap;
    start.next->size = address - (size_t)start.next;
    start.next->next = end;
    free_bytes = start.next->size;
    minimum_free_bytes = free_bytes;
    allocated_bit = (size_t)1U << (sizeof(size_t) * heapBITS_PER_BYTE - 1U);
}

static void insert_free_block(BlockLink_t *block) {
    BlockLink_t *iterator;
    for (iterator = &start; iterator->next < block; iterator = iterator->next) {}

    if ((uint8_t *)iterator + iterator->size == (uint8_t *)block) {
        iterator->size += block->size;
        block = iterator;
    }

    if ((uint8_t *)block + block->size == (uint8_t *)iterator->next) {
        if (iterator->next != end) {
            block->size += iterator->next->size;
            block->next = iterator->next->next;
        } else {
            block->next = end;
        }
    } else {
        block->next = iterator->next;
    }

    if (iterator != block) {
        iterator->next = block;
    }
}

#if (configUSE_HEAP_SCHEME == 4) || defined(USE_FreeRTOS_HEAP_4)
void vPortGetHeapStats(HeapStats_t *stats) {
    size_t blocks = 0U;
    size_t largest = 0U;
    size_t smallest = portMAX_DELAY;

    vTaskSuspendAll();
    BlockLink_t *block = start.next;
    if (block != NULL) {
        while (block != end) {
            blocks++;
            if (block->size > largest) largest = block->size;
            if (block->size < smallest) smallest = block->size;
            block = block->next;
        }
    }
    (void)xTaskResumeAll();

    stats->xSizeOfLargestFreeBlockInBytes = largest;
    stats->xSizeOfSmallestFreeBlockInBytes = blocks == 0U ? 0U : smallest;
    stats->xNumberOfFreeBlocks = blocks;
    taskENTER_CRITICAL();
    stats->xAvailableHeapSpaceInBytes = free_bytes;
    stats->xNumberOfSuccessfulAllocations = successful_allocations;
    stats->xNumberOfSuccessfulFrees = successful_frees;
    stats->xMinimumEverFreeBytesRemaining = minimum_free_bytes;
    taskEXIT_CRITICAL();
}
#endif
