/**
 * @file thread_safe_gpio.h
 *
 * @brief CAuDri - Thread-safe GPIO EXTI callback dispatcher
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback invoked directly from HAL_GPIO_EXTI_Callback.
 *
 * Implementations run in interrupt context and must not block or call APIs that
 * are not ISR-safe.
 *
 * @param port GPIO port registered for the EXTI line.
 * @param pin Single GPIO pin mask that triggered the interrupt.
 * @param context Application-owned callback context.
 */
typedef void (*GPIO_EXTI_Callback)(GPIO_TypeDef* port, uint16_t pin, void* context);

/**
 * @brief Register one callback for an EXTI line from thread context.
 *
 * STM32 EXTI hardware permits only one GPIO port per numbered EXTI line, so a
 * second callback using the same pin number is rejected even if its port is
 * different. GPIO edge selection, pull configuration, NVIC setup, and the IRQ
 * handler's HAL_GPIO_EXTI_IRQHandler call must already be configured.
 *
 * @param port GPIO port associated with the EXTI line.
 * @param pin Single GPIO pin mask from GPIO_PIN_0 through GPIO_PIN_15.
 * @param callback ISR-safe callback to invoke.
 * @param context Application-owned context passed to the callback.
 * @return true when the line was registered, otherwise false.
 */
bool GPIO_EXTI_RegisterCallback(GPIO_TypeDef* port, uint16_t pin, GPIO_EXTI_Callback callback, void* context);

/**
 * @brief Remove a previously registered EXTI callback from thread context.
 *
 * All registration arguments must match the active slot, preventing one owner
 * from accidentally unregistering another component's callback.
 *
 * @param port GPIO port used during registration.
 * @param pin GPIO pin mask used during registration.
 * @param callback Callback used during registration.
 * @param context Context used during registration.
 * @return true when the matching callback was removed, otherwise false.
 */
bool GPIO_EXTI_UnregisterCallback(GPIO_TypeDef* port, uint16_t pin, GPIO_EXTI_Callback callback, void* context);

#ifdef __cplusplus
}
#endif
