/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define HALT_IF_DEBUGGING()                                 \
  do {                                                      \
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) { \
      __asm volatile("bkpt 1");                             \
    }                                                       \
  } while (0)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */


    /**
      * CAuDri - Log the stack overflow error and halt the system
      */
    vTaskSuspendAll();

    LogInline(" ");
    LogInline("------ Stack Overflow in Task: %s ------", pcTaskName);
    LogInline(" ");
    LogInline("FreeRTOS: Stack overflow detected in task '%s'.", pcTaskName);
    LogInline("FreeRTOS: Halting system.");
    LogInline(" ");

    // Signalize error state by blinking the red onboard LED

    HALT_IF_DEBUGGING();

    while(1){
        HAL_GPIO_TogglePin(DEBUG_LED_RED_GPIO_Port, DEBUG_LED_RED_Pin);
        HAL_Delay(100);
    };
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */

    /**
      * CAuDri - Log the malloc failure
      */
    vTaskSuspendAll();

    LogInline(" ");
    LogInline("------ Memory Allocation Failed ------");
    LogInline(" ");
    LogInline("FreeRTOS: Memory allocation failed in pvPortMalloc(), possibly out of heap memory.");
    LogInline("FreeRTOS: Halting system.");
    LogInline(" ");

    extern void vPortGetHeapStats( HeapStats_t *pxHeapStats );
    HeapStats_t xHeapStats;

    vPortGetHeapStats( &xHeapStats );

    LogInline("FreeRTOS Heap Statistics:");
    LogInline("Total heap size: %d", configTOTAL_HEAP_SIZE);
    LogInline("Available heap space: %d", xHeapStats.xAvailableHeapSpaceInBytes);
    LogInline("Largest free block: %d", xHeapStats.xSizeOfLargestFreeBlockInBytes);
    LogInline("Smallest free block: %d", xHeapStats.xSizeOfSmallestFreeBlockInBytes);
    LogInline("Number of free blocks: %d", xHeapStats.xNumberOfFreeBlocks);
    LogInline("Minimum ever free bytes remaining: %d", xHeapStats.xMinimumEverFreeBytesRemaining);
    LogInline("Number of successful allocations: %d", xHeapStats.xNumberOfSuccessfulAllocations);
    LogInline("Number of successful frees: %d", xHeapStats.xNumberOfSuccessfulFrees);

    vTaskSuspendAll();

    // Signalize error state by blinking the blue onboard LED

    HALT_IF_DEBUGGING();

    while(1){
        HAL_GPIO_TogglePin(DEBUG_LED_BLUE_GPIO_Port, DEBUG_LED_BLUE_Pin);
        HAL_Delay(100);
    };
}
/* USER CODE END 5 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
