/**
 * @file pulse_animation.hpp
 *
 * @brief CAuDri - Pulse animation for lights
 *
 * This animation will pulse the brightness of all lights in and out
 * over a specified period and duration. Same as BlinkAnimation but the lights will smoothly transition between the off state and the set color.
 * 
 * A color can be specified for the pulsing; if no color is specified, the lights will use their existing colors.
 */
#pragma once

#include <algorithm>
#include <array>

#include "animation.hpp"
#include "light.hpp"
#include "logger.h"

/**
 * @brief Pulse animation class
 *
 * This animation will pulse all lights in and out for a specified duration
 * with a specified pulse period.
 */
class PulseAnimation : public Animation {
   public:
    PulseAnimation(uint32_t duration_ms = osWaitForever,
                   uint32_t pulse_period_ms = DEFAULT_PULSE_PERIOD_MS,
                   uint32_t on_time_ms = DEFAULT_ON_TIME_MS,
                   uint32_t rise_time_ms = DEFAULT_RISE_TIME_MS,
                   Color default_color = COLOR_OFF);

   protected:
    bool run() override;

   private:
    constexpr static uint32_t DEFAULT_PULSE_PERIOD_MS = 1000;  // Default pulse period in milliseconds
    constexpr static uint32_t DEFAULT_ON_TIME_MS = 200;        // Default on time in milliseconds
    constexpr static uint32_t DEFAULT_RISE_TIME_MS = 200;      //   Default rise time in milliseconds
    constexpr static uint32_t TRANSITION_UPDATE_PERIOD_MS = 10;  // Update period for brightness transitions in milliseconds (50 Hz)

    uint32_t duration_ms;

    uint32_t pulse_period_ms;
    uint32_t on_time_ms;
    uint32_t rise_time_ms;
    Color default_color;
};

/**
 * @brief Construct a PulseAnimation with a fixed-size array of Light pointers
 * *
 * @param duration_ms Total duration of the pulse animation in milliseconds (default: infinite)
 * @param pulse_period_ms Total period of one pulse cycle in milliseconds (default: 1000 ms)
 * @param on_time_ms Time in milliseconds that the lights stay at full brightness during each pulse (default: 200 ms)
 * @param rise_time_ms Time in milliseconds for the lights to rise and fall in brightness (default: 200 ms)
 * @param default_color Color to use when pulsing lights (default: COLOR_OFF to use existing colors)
 */
PulseAnimation::PulseAnimation(uint32_t duration_ms, uint32_t pulse_period_ms, uint32_t on_time_ms, uint32_t rise_time_ms, Color default_color)
    : Animation() {
    if (pulse_period_ms == 0) {
        LogWarning("PulseAnimation: Pulse period cannot be zero, setting to default %lu ms", DEFAULT_PULSE_PERIOD_MS);
        pulse_period_ms = DEFAULT_PULSE_PERIOD_MS;
    }
    if (on_time_ms >= pulse_period_ms) {
        LogWarning("PulseAnimation: on_time_ms (%lu ms) must be less than pulse_period_ms (%lu ms), adjusting on time",
                   on_time_ms,
                   pulse_period_ms);
        on_time_ms = pulse_period_ms / 2;
    }
    if (rise_time_ms * 2 >= (pulse_period_ms - on_time_ms)) {
        rise_time_ms = (pulse_period_ms - on_time_ms) / 2;
        LogWarning("PulseAnimation: rise_time_ms too long, adjusting to %lu ms", rise_time_ms);
    }

    this->duration_ms = duration_ms;
    this->pulse_period_ms = pulse_period_ms;
    this->on_time_ms = on_time_ms;
    this->rise_time_ms = rise_time_ms;
    this->default_color = default_color;
}

/**
 * @brief Run the pulse animation
 *
 * This function will pulse all lights in and out for a specified duration
 * with a specified pulse period.
 */
bool PulseAnimation::run() {
    uint32_t start_time = osKernelGetTickCount();
    uint32_t end_time = (duration_ms == osWaitForever) ? osWaitForever : start_time + duration_ms;

    bool use_default_color = (default_color != COLOR_OFF);

    uint32_t off_time_ms = pulse_period_ms - on_time_ms - (2 * rise_time_ms);
    if (off_time_ms >= pulse_period_ms) {
        LogWarning("PulseAnimation: off_time_ms (%lu ms) is too long, adjusting to 0 ms", off_time_ms);
        off_time_ms = 0;
    }

    uint32_t transition_step_count = rise_time_ms / TRANSITION_UPDATE_PERIOD_MS;
    transition_step_count = std::max(transition_step_count, 1ul);  // Ensure at least one step

    struct ColorHSV {
        uint16_t h;
        uint8_t s;
        uint8_t v;
    };
 
    ColorHSV initial_colors_hsv[light_count];
    for (size_t i = 0; i < light_count; ++i) {
        Color color = use_default_color ? default_color : lights[i]->getColor();

        uint16_t h;
        uint8_t s, v;
        color.toHSV(&h, &s, &v);
        initial_colors_hsv[i] = {h, s, v};

        // Start with lights off
        lights[i]->turnOff();
    }

    while (osKernelGetTickCount() < end_time) {
        // Transition from 0% to 100% brightness
        for (uint32_t step = 0; step <= transition_step_count; ++step) {
            for (size_t i = 0; i < light_count; ++i) {
                uint8_t target_val = initial_colors_hsv[i].v;
                uint8_t new_val = static_cast<uint8_t>((static_cast<uint32_t>(target_val) * step) / transition_step_count);

                Color new_color = Color::fromHSV(initial_colors_hsv[i].h, initial_colors_hsv[i].s, new_val);
                lights[i]->setColor(new_color, true);
            }
            if (exitOnDelay(TRANSITION_UPDATE_PERIOD_MS)) {
                return false;
            }
        }

        // Exiting because the duration has been reached will be checked after each full transition phase
        if (osKernelGetTickCount() >= end_time) {
            break;
        }

        // Stay at full brightness
        if (exitOnDelay(on_time_ms)) {
            return false;
        }

        if (osKernelGetTickCount() >= end_time) {
            break;
        }

        // Transition from 100% to 0% brightness
        for (uint32_t step = 0; step <= transition_step_count; ++step) {
            for (size_t i = 0; i < light_count; ++i) {
                uint8_t start_val = initial_colors_hsv[i].v;
                uint8_t new_val = static_cast<uint8_t>(
                    (static_cast<uint32_t>(start_val) * (transition_step_count - step)) / transition_step_count);
                Color new_color = Color::fromHSV(initial_colors_hsv[i].h, initial_colors_hsv[i].s, new_val);
                lights[i]->setColor(new_color, true);
            }
            if (exitOnDelay(TRANSITION_UPDATE_PERIOD_MS)) {
                return false;
            }
        }

        if (osKernelGetTickCount() >= end_time) {
            break;
        }

        // Stay off for the remaining pulse period
        if (exitOnDelay(off_time_ms)) {
            return false;
        }
    }
    return true;
}