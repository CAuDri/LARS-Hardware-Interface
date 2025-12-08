/**
 * @file gpio_light.cpp
 * 
 * @brief CAuDri - Implementation for controlling single-color GPIO-controlled lights
 */
#include "gpio_light.hpp"

/**
 * @brief Constructor for the GPIOLight class
 *
 * @param gpio_port GPIO port where the light is connected
 * @param gpio_pin GPIO pin where the light is connected
 * @param active_high Whether the light is active high (true) or active low (false)
 */
GPIOLight::GPIOLight(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, Color led_color, bool active_high)
    : gpio_port(gpio_port), gpio_pin(gpio_pin), led_color(led_color), active_high(active_high) {
    turnOff();
}

/**
 * @brief Get the number of LEDs controlled by this light
 * 
 * @return Number of LEDs (1)
 */
uint32_t GPIOLight::getLEDCount() const {
    return 1;
}

/**
 * @brief Turn on the light
 * 
 * @return true if successful, false otherwise
 */
bool GPIOLight::turnOn() {
    if (isNotLockOwner()) {
        return false;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_pin, active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    state = true;
    return true;
}

/**
 * @brief Turn off the light
 * 
 * @return true if successful, false otherwise
 */
bool GPIOLight::turnOff() {
    if (isNotLockOwner()) {
        return false;
    }
    HAL_GPIO_WritePin(gpio_port, gpio_pin, active_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
    state = false;
    return true;
}

/**
 * @brief Set the color of the light (single-color light, so color is ignored)
 * 
 * @param color Color to set (ignored)
 * @param turn_on Whether to turn on the light after setting the color
 * @return true if successful, false otherwise
 */
bool GPIOLight::setColor(const Color& color, bool turn_on) {
    if (turn_on) {
        return turnOn();
    } else {
        return turnOff();
    }
}

/**
 * @brief Set the brightness of the light (single-color light, so brightness is ignored)
 * 
 * LED will be turned on if brightness > 0
 * 
 * @param brightness Brightness to set (ignored)
 * @return true if successful, false otherwise
 */
bool GPIOLight::setBrightness(uint8_t brightness) {
    if (brightness > 0) {
        return turnOn();
    } else {
        return turnOff();
    }
}

/**
 * @brief Check if the light is currently on
 * 
 * @return true if the light is on, false otherwise
 */
bool GPIOLight::isOn() const {
    return state;
}

/**
 * @brief Check if the light is currently off
 * 
 * @return true if the light is off, false otherwise
 */
bool GPIOLight::isOff() const {
    return !state;
}

/**
 * @brief Get the current color of the light (single-color light, so return white if on, black if off)
 * 
 * @return Current color of the light
 */
Color GPIOLight::getColor() const {
    return state ? led_color : COLOR_OFF;
}

/**
 * @brief Turn on the light at the specified index (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @return true if successful, false otherwise
 */
bool GPIOLight::turnOn(uint32_t index) {
    if (index != 0) {
        return false;
    }
    return turnOn();
}

/**
 * @brief Turn off the light at the specified index (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @return true if successful, false otherwise
 */
bool GPIOLight::turnOff(uint32_t index) {
    if (index != 0) {
        return false;
    }
    return turnOff();
}

/**
 * @brief Set the color of the light at the specified index (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @param color Color to set (ignored)
 * @param turn_on Whether to turn on the light after setting the color
 * @return true if successful, false otherwise
 */
bool GPIOLight::setColor(uint32_t index, const Color& color, bool turn_on) {
    if (index != 0) {
        return false;
    }
    return setColor(color, turn_on);
}

/**
 * @brief Set the brightness of the light at the specified index (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @param brightness Brightness to set (ignored)
 * @return true if successful, false otherwise
 */
bool GPIOLight::setBrightness(uint32_t index, uint8_t brightness) {
    if (index != 0) {
        return false;
    }
    return setBrightness(brightness);
}

/**
 * @brief Check if the light at the specified index is currently on (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @return true if the light is on, false otherwise
 */
bool GPIOLight::isOn(uint32_t index) const {
    if (index != 0) {
        return false;
    }
    return isOn();
}

/**
 * @brief Check if the light at the specified index is currently off (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @return true if the light is off, false otherwise
 */
bool GPIOLight::isOff(uint32_t index) const {
    if (index != 0) {
        return false;
    }
    return isOff();
}   

/**
 * @brief Get the current color of the light at the specified index (only index 0 is valid)
 * 
 * @param index Index of the light (must be 0)
 * @return Current color of the light
 */
Color GPIOLight::getColor(uint32_t index) const {
    if (index != 0) {
        return COLOR_OFF;
    }
    return getColor();
}