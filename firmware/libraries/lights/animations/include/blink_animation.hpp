/**
 * @file blink_animation.hpp
 *
 * @brief CAuDri - Simple blink animation for lights
 * 
 * This animation will blink all lights on and off for a specified number of times
 * with a specified delay between states.
 * 
 * Colors can be specified for the "on" state; if no color is specified, the lights will use their existing colors.
 */
#pragma once

#include "animation.hpp"
#include "light.hpp"
#include "logger.h"

/**
 * @brief Blink animation class
 *
 * This animation will blink all lights on and off for a specified number of times
 * with a specified delay between states.
 */
class BlinkAnimation : public Animation {
   public:
    BlinkAnimation(uint32_t duration_ms = osWaitForever,
                   uint32_t blink_period_ms = DEFAULT_BLINK_PERIOD_MS,
                   float duty_cycle = DEFAULT_DUTY_CYCLE,
                   Color default_color = COLOR_OFF);

   protected:
    bool run() override;

   private:
    constexpr static uint32_t DEFAULT_BLINK_PERIOD_MS = 500;  // Default blink period in milliseconds
    constexpr static float DEFAULT_DUTY_CYCLE = 0.5f;         // Default duty cycle (on time / total period)

    uint32_t duration_ms;

    uint32_t blink_period_ms;
    float duty_cycle;
    Color default_color;
};

/**
 * @brief Construct a BlinkAnimation with a fixed-size array of Light pointers
 *
 * @param duration_ms Total duration of the blink animation in milliseconds (default: infinite)
 * @param blink_period_ms Total period of one blink cycle in milliseconds (default: 500 ms)
 * @param duty_cycle Fraction of the blink period that the lights are on (0.0 to 1.0, default: 0.5)
 * @param default_color Color to use when turning lights on (default: COLOR_OFF to use existing colors)
 */
BlinkAnimation::BlinkAnimation(uint32_t duration_ms, uint32_t blink_period_ms, float duty_cycle, Color default_color)
    : Animation() {
    if (blink_period_ms == 0) {
        LogWarning("BlinkAnimation: Blink period cannot be zero, setting to default %lu ms", DEFAULT_BLINK_PERIOD_MS);
        blink_period_ms = DEFAULT_BLINK_PERIOD_MS;
    }
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
        LogWarning("BlinkAnimation: duty_cycle (%.2f) must be between 0.0 and 1.0, setting to default %.2f", duty_cycle, DEFAULT_DUTY_CYCLE);
        duty_cycle = DEFAULT_DUTY_CYCLE;
    }

    this->duration_ms = duration_ms;
    this->blink_period_ms = blink_period_ms;
    this->duty_cycle = duty_cycle;
    this->default_color = default_color;
}

/**
 * @brief Run the blink animation
 *
 * This function will blink all lights on and off for a specified number of times
 * with a specified delay between states.
 *
 * @return true if the animation ran successfully, false if it was stopped midway
 */
bool BlinkAnimation::run() {
    uint32_t start_time = osKernelGetTickCount();

    uint32_t on_time = static_cast<uint32_t>(blink_period_ms * duty_cycle);
    uint32_t off_time = blink_period_ms - on_time;

    bool use_default_color = (default_color != COLOR_OFF);

    while (true) {
        // Check if total duration has been exceeded
        if (osKernelGetTickCount() - start_time >= duration_ms) {
            break;
        }

        // Turn all lights on
        for (size_t i = 0; i < light_count; ++i) {
            if (use_default_color) {
                lights[i]->setColor(default_color);
            } else {
                lights[i]->turnOn();
            }
        }
        if (exitOnDelay(on_time)) {
            return false;
        }

        if (osKernelGetTickCount() - start_time >= duration_ms) {
            break;
        }

        // Turn all lights off
        for (size_t i = 0; i < light_count; ++i) {
            lights[i]->turnOff();
        }
        if (exitOnDelay(off_time)) {
            return false;
        }
    }

    return true;
}