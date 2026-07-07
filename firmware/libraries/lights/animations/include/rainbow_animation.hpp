/**
 * @file rainbow_animation.hpp
 *
 * @brief Moving rainbow animation for indexed lights.
 *
 * This ports the KITcar rainbow idea to the portable Light interface. Each
 * indexed LED receives a hue offset and the base hue sweeps over time.
 */
#pragma once

#include <algorithm>
#include <array>

#include "animation.hpp"
#include "light.hpp"
#include "logger.h"

class RainbowAnimation : public Animation {
   public:
    RainbowAnimation(uint32_t duration_ms = osWaitForever,
                     uint16_t start_hue_deg = DEFAULT_START_HUE_DEG,
                     uint16_t stop_hue_deg = DEFAULT_STOP_HUE_DEG,
                     uint32_t update_period_ms = DEFAULT_UPDATE_PERIOD_MS,
                     uint16_t hue_step_deg = DEFAULT_HUE_STEP_DEG,
                     uint8_t saturation = DEFAULT_SATURATION,
                     uint8_t value = DEFAULT_VALUE);

   protected:
    bool run() override;

   private:
    constexpr static uint16_t DEFAULT_START_HUE_DEG = 0;
    constexpr static uint16_t DEFAULT_STOP_HUE_DEG = 359;
    constexpr static uint32_t DEFAULT_UPDATE_PERIOD_MS = 20;
    constexpr static uint16_t DEFAULT_HUE_STEP_DEG = 6;
    constexpr static uint8_t DEFAULT_SATURATION = 255;
    constexpr static uint8_t DEFAULT_VALUE = 255;

    uint32_t duration_ms;
    uint16_t start_hue_deg;
    uint16_t stop_hue_deg;
    uint32_t update_period_ms;
    uint16_t hue_step_deg;
    uint8_t saturation;
    uint8_t value;

    void renderFrame(uint16_t base_hue_deg);
    void renderFrameIndexed(Light* light, uint32_t led_count, uint16_t base_hue_deg);
    uint16_t wrapHue(uint32_t hue_deg) const;
};

inline RainbowAnimation::RainbowAnimation(uint32_t duration_ms,
                                          uint16_t start_hue_deg,
                                          uint16_t stop_hue_deg,
                                          uint32_t update_period_ms,
                                          uint16_t hue_step_deg,
                                          uint8_t saturation,
                                          uint8_t value)
    : Animation(),
      duration_ms(duration_ms),
      start_hue_deg(std::min<uint16_t>(start_hue_deg, DEFAULT_STOP_HUE_DEG)),
      stop_hue_deg(std::min<uint16_t>(stop_hue_deg, DEFAULT_STOP_HUE_DEG)),
      update_period_ms(update_period_ms),
      hue_step_deg(hue_step_deg),
      saturation(saturation),
      value(value) {
    if (this->stop_hue_deg <= this->start_hue_deg) {
        LogWarning("RainbowAnimation: invalid hue range, using full hue wheel");
        this->start_hue_deg = DEFAULT_START_HUE_DEG;
        this->stop_hue_deg = DEFAULT_STOP_HUE_DEG;
    }
    if (update_period_ms == 0) {
        LogWarning("RainbowAnimation: update_period_ms cannot be zero, using %lu ms", DEFAULT_UPDATE_PERIOD_MS);
        this->update_period_ms = DEFAULT_UPDATE_PERIOD_MS;
    }
    if (hue_step_deg == 0) {
        LogWarning("RainbowAnimation: hue_step_deg cannot be zero, using %u deg", DEFAULT_HUE_STEP_DEG);
        this->hue_step_deg = DEFAULT_HUE_STEP_DEG;
    }
}

inline bool RainbowAnimation::run() {
    const uint32_t start_time = osKernelGetTickCount();
    uint16_t base_hue_deg = start_hue_deg;

    while (duration_ms == osWaitForever || osKernelGetTickCount() - start_time < duration_ms) {
        renderFrame(base_hue_deg);
        base_hue_deg = wrapHue(base_hue_deg + hue_step_deg);

        if (exitOnDelay(update_period_ms)) {
            return false;
        }
    }

    return true;
}

inline void RainbowAnimation::renderFrame(uint16_t base_hue_deg) {
    std::array<Color, ANIMATION_MAX_LED_CAPACITY> frame{};

    for (size_t light_index = 0; light_index < light_count; ++light_index) {
        Light* light = lights[light_index];
        const uint32_t led_count = light->getLEDCount();

        if (led_count > frame.size()) {
            renderFrameIndexed(light, led_count, base_hue_deg);
            continue;
        }

        for (uint32_t led_index = 0; led_index < led_count; ++led_index) {
            const uint16_t hue = wrapHue(base_hue_deg + led_index * hue_step_deg);
            frame[led_index] = Color::fromHSV(hue, saturation, value);
        }

        light->setColors(frame.data(), led_count, true);
    }
}

inline void RainbowAnimation::renderFrameIndexed(Light* light, uint32_t led_count, uint16_t base_hue_deg) {
    for (uint32_t led_index = 0; led_index < led_count; ++led_index) {
        const uint16_t hue = wrapHue(base_hue_deg + led_index * hue_step_deg);
        light->setColor(led_index, Color::fromHSV(hue, saturation, value), true);
    }
}

inline uint16_t RainbowAnimation::wrapHue(uint32_t hue_deg) const {
    const uint32_t hue_span = stop_hue_deg - start_hue_deg + 1;
    return static_cast<uint16_t>(start_hue_deg + ((hue_deg - start_hue_deg) % hue_span));
}
