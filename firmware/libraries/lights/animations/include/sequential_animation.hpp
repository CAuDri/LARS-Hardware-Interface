/**
 * @file sequential_animation.hpp
 *
 * @brief Sequential fill animation for indexed lights.
 *
 * This animation lights LEDs one after another, optionally holding the full
 * strip on before clearing it again. It is the portable equivalent of the
 * KITcar "Audi" blinker effect.
 */
#pragma once

#include <algorithm>
#include <array>

#include "animation.hpp"
#include "light.hpp"
#include "logger.h"

class SequentialAnimation : public Animation {
   public:
    SequentialAnimation(uint32_t duration_ms = osWaitForever,
                        uint32_t rise_time_ms = DEFAULT_RISE_TIME_MS,
                        uint32_t on_time_ms = DEFAULT_ON_TIME_MS,
                        uint32_t off_time_ms = DEFAULT_OFF_TIME_MS,
                        Color color = COLOR_ORANGE,
                        bool inverted = false);

   protected:
    bool run() override;

   private:
    constexpr static uint32_t DEFAULT_RISE_TIME_MS = 250;
    constexpr static uint32_t DEFAULT_ON_TIME_MS = 250;
    constexpr static uint32_t DEFAULT_OFF_TIME_MS = 500;
    constexpr static uint32_t MIN_STEP_TIME_MS = 1;

    uint32_t duration_ms;
    uint32_t rise_time_ms;
    uint32_t on_time_ms;
    uint32_t off_time_ms;
    Color color;
    bool inverted;

    void setSequentialState(uint32_t active_count);
    void setSequentialStateIndexed(Light* light, uint32_t led_count, uint32_t active_count);
    void clearLights();
};

/**
 * @brief Sequential animation
 *
 * @param duration_ms Total duration of the animation in milliseconds. Use osWaitForever for infinite duration.
 * @param rise_time_ms Time taken to light up all LEDs in milliseconds.
 * @param on_time_ms Time to keep all LEDs on after the rise time in milliseconds.
 * @param off_time_ms Time to keep all LEDs off after the on time in milliseconds.
 * @param color Color to use for the animation.
 * @param inverted If true, the animation will light LEDs in reverse order.
 */
inline SequentialAnimation::SequentialAnimation(uint32_t duration_ms,
                                                uint32_t rise_time_ms,
                                                uint32_t on_time_ms,
                                                uint32_t off_time_ms,
                                                Color color,
                                                bool inverted)
    : Animation(),
      duration_ms(duration_ms),
      rise_time_ms(rise_time_ms),
      on_time_ms(on_time_ms),
      off_time_ms(off_time_ms),
      color(color),
      inverted(inverted) {
    if (rise_time_ms == 0) {
        LogWarning("SequentialAnimation: rise_time_ms cannot be zero, using %lu ms", DEFAULT_RISE_TIME_MS);
        this->rise_time_ms = DEFAULT_RISE_TIME_MS;
    }
}

inline bool SequentialAnimation::run() {
    const uint32_t start_time = osKernelGetTickCount();

    while (duration_ms == osWaitForever || osKernelGetTickCount() - start_time < duration_ms) {
        uint32_t max_led_count = 0;
        for (size_t light_index = 0; light_index < light_count; ++light_index) {
            max_led_count = std::max(max_led_count, lights[light_index]->getLEDCount());
        }

        if (max_led_count == 0) {
            return true;
        }

        const uint32_t step_time_ms = std::max(rise_time_ms / max_led_count, MIN_STEP_TIME_MS);
        for (uint32_t active_count = 1; active_count <= max_led_count; ++active_count) {
            setSequentialState(active_count);
            if (exitOnDelay(step_time_ms)) {
                return false;
            }
        }

        if (exitOnDelay(on_time_ms)) {
            return false;
        }

        clearLights();
        if (exitOnDelay(off_time_ms)) {
            return false;
        }
    }

    return true;
}

inline void SequentialAnimation::setSequentialState(uint32_t active_count) {
    std::array<Color, ANIMATION_MAX_LED_CAPACITY> frame{};

    for (size_t light_index = 0; light_index < light_count; ++light_index) {
        Light* light = lights[light_index];
        const uint32_t led_count = light->getLEDCount();

        if (led_count > frame.size()) {
            setSequentialStateIndexed(light, led_count, active_count);
            continue;
        }

        for (uint32_t led_index = 0; led_index < led_count; ++led_index) {
            const uint32_t logical_index = inverted ? (led_count - 1 - led_index) : led_index;
            frame[logical_index] = (led_index < active_count) ? color : COLOR_OFF;
        }

        light->setColors(frame.data(), led_count, true);
    }
}

inline void SequentialAnimation::setSequentialStateIndexed(Light* light, uint32_t led_count, uint32_t active_count) {
    for (uint32_t led_index = 0; led_index < led_count; ++led_index) {
        const uint32_t logical_index = inverted ? (led_count - 1 - led_index) : led_index;
        if (led_index < active_count) {
            light->setColor(logical_index, color, true);
        } else {
            light->turnOff(logical_index);
        }
    }
}

inline void SequentialAnimation::clearLights() {
    for (size_t light_index = 0; light_index < light_count; ++light_index) {
        lights[light_index]->turnOff();
    }
}
