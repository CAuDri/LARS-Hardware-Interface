/**
 * @file gpio_light.hpp
 * 
 * @brief CAuDri - Interface for controlling single-color GPIO-controlled lights
 */
#pragma once

#include "light.hpp"
#include "main.h"

/**
 * @brief Implementation of the Light interface for a single GPIO controlled LED
 */
class GPIOLight : public Light {
    public:
    GPIOLight(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, Color led_color = COLOR_WHITE, bool active_high = true);
    ~GPIOLight() override = default;

    uint32_t getLEDCount() const override;

    bool turnOn() override;
    bool turnOff() override;
    bool setColor(const Color& color, bool turn_on = true) override;
    bool setBrightness(uint8_t brightness) override;

    bool isOn() const override;
    bool isOff() const override;
    Color getColor() const override;

    bool turnOn(uint32_t index) override;
    bool turnOff(uint32_t index) override;
    bool setColor(uint32_t index, const Color& color, bool turn_on = true) override;
    bool setBrightness(uint32_t index, uint8_t brightness) override;

    bool isOn(uint32_t index) const override;
    bool isOff(uint32_t index) const override;
    Color getColor(uint32_t index) const override;

    private:
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
    Color led_color;
    bool active_high;

    bool state = false;  // Current state of the light (on/off)
};