/**
 * @file thread_safe_gpio.c
 *
 * @brief CAuDri - Thread-safe GPIO EXTI callback dispatcher
 */
#include "thread_safe_gpio.h"

#include <stddef.h>

#define GPIO_EXTI_LINE_COUNT 16U

typedef struct {
    GPIO_TypeDef* port;
    GPIO_EXTI_Callback callback;
    void* context;
} GPIO_EXTI_Slot;

// EXTI line numbers map directly to GPIO pin bit positions. Indexing by line
// avoids a search in the ISR and naturally enforces the STM32 one-port-per-line rule.
static GPIO_EXTI_Slot exti_slots[GPIO_EXTI_LINE_COUNT];

static int32_t get_exti_line(uint16_t pin) {
    if (pin == 0U || pin > GPIO_PIN_15 || (pin & (pin - 1U)) != 0U) {
        return -1;
    }

    for (uint32_t line = 0; line < GPIO_EXTI_LINE_COUNT; ++line) {
        if (pin == (uint16_t)(1U << line)) {
            return (int32_t)line;
        }
    }
    return -1;
}

static uint32_t enter_critical(void) {
    // Preserve PRIMASK so registration is safe both before and after the RTOS
    // starts without accidentally enabling interrupts disabled by the caller.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical(uint32_t primask) {
    if (primask == 0U) {
        __enable_irq();
    }
}

bool GPIO_EXTI_RegisterCallback(GPIO_TypeDef* port, uint16_t pin, GPIO_EXTI_Callback callback, void* context) {
    const int32_t line = get_exti_line(pin);
    if (line < 0 || port == NULL || callback == NULL || __get_IPSR() != 0U) {
        return false;
    }

    const uint32_t primask = enter_critical();
    GPIO_EXTI_Slot* slot = &exti_slots[line];
    if (slot->callback != NULL) {
        exit_critical(primask);
        return false;
    }

    // Publish the callback last. The ISR therefore cannot observe a callback
    // paired with partially initialized port or context fields.
    slot->port = port;
    slot->context = context;
    slot->callback = callback;
    exit_critical(primask);
    return true;
}

bool GPIO_EXTI_UnregisterCallback(GPIO_TypeDef* port, uint16_t pin, GPIO_EXTI_Callback callback, void* context) {
    const int32_t line = get_exti_line(pin);
    if (line < 0 || __get_IPSR() != 0U) {
        return false;
    }

    const uint32_t primask = enter_critical();
    GPIO_EXTI_Slot* slot = &exti_slots[line];
    if (slot->port != port || slot->callback != callback || slot->context != context) {
        exit_critical(primask);
        return false;
    }

    // Withdraw the callback first for the inverse reason used during registration.
    slot->callback = NULL;
    slot->context = NULL;
    slot->port = NULL;
    exit_critical(primask);
    return true;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    const int32_t line = get_exti_line(pin);
    if (line < 0) {
        return;
    }

    const GPIO_EXTI_Slot* slot = &exti_slots[line];
    if (slot->callback != NULL) {
        slot->callback(slot->port, pin, slot->context);
    }
}
