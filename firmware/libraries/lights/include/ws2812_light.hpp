/**
 * @file ws2812_light.hpp
 *
 * @brief CAuDri - Interface for controlling WS2812 LED strips
 * 
 * Implements the Light interface for WS2812 LED strips using a WS2812Driver.
 * Allows for controlling a subset of LEDs in the strip with individual colors and brightness levels.
 * 
 * While the WS2812 driver itself represents the entire LED strip, this class allows for segmenting
 * the strip into smaller logical segments that can be controlled independently.
 *
 */
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "light.hpp"
#include "ws2812.hpp"

constexpr uint8_t WS2812_MIN_BRIGHTNESS = 5;

/**
 * @brief Implementation of the Light interface for WS2812 LED sub-strips
 *
 * @tparam LED_COUNT Number of LEDs in the strip
 */
template <size_t LED_COUNT>
class WS2812Light : public Light {
   public:
    WS2812Light(WS2812Driver& driver, size_t start_index = 0, bool invert = false);
    ~WS2812Light() override = default;

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
    WS2812Driver& driver;
    size_t start_index;
    bool invert_direction = false;

    bool error_flag = false;

    // Frame buffer to store the current color state of each LED
    std::array<Color, LED_COUNT> frame_buffer{};

    // Bitmask to track which LEDs are currently on
    // std::array<uint32_t, (LED_COUNT + 31) / 32> on_mask{};
    std::bitset<LED_COUNT> on_mask{};

    // Masked buffer for updating the driver
    std::array<Color, LED_COUNT> masked_buffer{};

    bool getOnState(uint32_t index) const;
    void setOnState(uint32_t index, bool state);

    bool updateDriver();
};

/**
 * @brief Constructor for the WS2812Light class
 *
 * @tparam LED_COUNT Number of LEDs in the strip
 *
 * @param driver Reference to the WS2812 driver instance
 * @param start_index Starting index of the LED segment controlled by this light
 */
template <size_t LED_COUNT>
WS2812Light<LED_COUNT>::WS2812Light(WS2812Driver& driver, size_t start_index, bool invert)
    : driver(driver), start_index(start_index), invert_direction(invert) {
    if (start_index + LED_COUNT > driver.getLEDCount()) {
        LogError("WS2812Light: LED range (%u to %u) out of bounds for driver with %lu LEDs",
                 start_index,
                 start_index + LED_COUNT - 1,
                 driver.getLEDCount());
        error_flag = true;
    }
}

/**
 * @brief Get the number of LEDs controlled by this light
 *
 * @return Number of LEDs
 */
template <size_t LED_COUNT>
uint32_t WS2812Light<LED_COUNT>::getLEDCount() const {
    return LED_COUNT;
}

/**
 * @brief Get the on/off state of an LED at the specified index
 *
 * @param index Index of the LED
 * @return true if the LED is on, false if off
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::getOnState(uint32_t index) const {
    if (index >= LED_COUNT || error_flag) {
        return false;
    }
    return on_mask.test(index);
}

/**
 * @brief Set the on/off state of an LED at the specified index
 *
 * @param index Index of the LED
 * @param state true to set the LED on, false to set it off
 */
template <size_t LED_COUNT>
void WS2812Light<LED_COUNT>::setOnState(uint32_t index, bool state) {
    if (index >= LED_COUNT || error_flag) {
        return;
    }
    on_mask.set(index, state);
}

/**
 * @brief Update the WS2812 driver with the current frame buffer
 *
 * This function writes the colors from the frame buffer to the driver,
 * taking into account the on/off state of each LED.
 * The driver might block until the transmission is complete with a max timeout.
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::updateDriver() {
    if (error_flag) {
        return false;
    }
    // Depending on the on/off state, prepare the masked buffer
    // If the 'invert_direction' flag is set, we swap the order of LEDs
    for (size_t i = 0; i < LED_COUNT; ++i) {
        size_t index = invert_direction ? (LED_COUNT - 1 - i) : i;
        masked_buffer[i] = getOnState(index) ? frame_buffer[index] : Color(0, 0, 0);
    }
    // Write the masked buffer to the driver
    return driver.setColors(masked_buffer.data(), start_index, LED_COUNT);
}

/**
 * @brief Turn on all LEDs with the currently set colors
 *
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::turnOn() {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    on_mask.set();
    return updateDriver();
}

/**
 * @brief Turn off all LEDs
 *
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::turnOff() {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    on_mask.reset();
    return updateDriver();
}

/**
 * @brief Set the color for all LEDs
 *
 * @param color Color to set
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::setColor(const Color& color, bool turn_on) {
    if (isNotLockOwner() || error_flag) {
        LogDebug("WS2812Light: setColor called but lock not owned or error flag set");
        return false;
    }
    frame_buffer.fill(color);
    if (turn_on) {
        turnOn();
    }
    return updateDriver();
}

/**
 * @brief Set the brightness for all LEDs
 *
 * @param brightness Brightness level (0-255)
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::setBrightness(uint8_t brightness) {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    // Changing the brightness will be done by converting to HSV and back.
    // Some color information will be lost in this process, since it is not necessarily reversible.
    // The minimum brightness is clamped, to avoid loosing all stored color information.
    brightness = std::max(brightness, WS2812_MIN_BRIGHTNESS);
    for (size_t i = 0; i < LED_COUNT; ++i) {
        uint16_t hue;
        uint8_t saturation, value;
        frame_buffer[i].toHSV(&hue, &saturation, &value);
        value = brightness;
        frame_buffer[i] = Color::fromHSV(hue, saturation, value);
    }
    return updateDriver();
}

/**
 * @brief Check if *any* LEDs are currently on
 *
 * @return true if any LEDs are on, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::isOn() const {
    for (size_t i = 0; i < LED_COUNT; ++i) {
        if (on_mask.test(i)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Check if *all* LEDs are currently off
 *
 * @return true if all LEDs are off, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::isOff() const {
    return !isOn();
}

/**
 * @brief Get the color of the first LED
 *
 * "Getting the color" of a multi-LED strip is somewhat ambiguous, this implementation can be adjusted as needed.
 *
 * @return Color of the first LED
 */
template <size_t LED_COUNT>
Color WS2812Light<LED_COUNT>::getColor() const {
    static_assert(LED_COUNT > 0, "WS2812Light must have at least one LED");
    return frame_buffer[0];
}

/**
 * @brief Turn on the LED at the specified index
 *
 * @param index Index of the LED
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::turnOn(uint32_t index) {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    setOnState(index, true);
    return updateDriver();
}

/**
 * @brief Turn off the LED at the specified index
 *
 * @param index Index of the LED
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::turnOff(uint32_t index) {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    setOnState(index, false);
    return updateDriver();
}

/**
 * @brief Set the color of the LED at the specified index
 *
 * @param index Index of the LED
 * @param color Color to set
 * @param turn_on Whether to turn on the LED after setting the color
 *
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::setColor(uint32_t index, const Color& color, bool turn_on) {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    if (index >= LED_COUNT) {
        return false;
    }
    frame_buffer[index] = color;
    if (turn_on) {
        setOnState(index, true);
    }
    return updateDriver();
}

/**
 * @brief Set the brightness of the LED at the specified index
 *
 * @param index Index of the LED
 * @param brightness Brightness level (0-255)
 * @return true if successful, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::setBrightness(uint32_t index, uint8_t brightness) {
    if (isNotLockOwner() || error_flag) {
        return false;
    }
    if (index >= LED_COUNT) {
        return false;
    }
    // Changing the brightness will be done by converting to HSV and back.
    // Some color information will be lost in this process, since it is not necessarily reversible.
    // The minimum brightness is clamped, to avoid loosing all stored color information.
    brightness = std::max(brightness, WS2812_MIN_BRIGHTNESS);
    uint16_t hue;
    uint8_t saturation, value;
    frame_buffer[index].toHSV(&hue, &saturation, &value);
    value = brightness;
    frame_buffer[index] = Color::fromHSV(hue, saturation, value);
    return updateDriver();
}

/**
 * @brief Check if the LED at the specified index is on
 *
 * @param index Index of the LED
 * @return true if the LED is on, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::isOn(uint32_t index) const {
    return getOnState(index);
}

/**
 * @brief Check if the LED at the specified index is off
 *
 * @param index Index of the LED
 * @return true if the LED is off, false otherwise
 */
template <size_t LED_COUNT>
bool WS2812Light<LED_COUNT>::isOff(uint32_t index) const {
    return !getOnState(index);
}

/**
 * @brief Get the color of the LED at the specified index
 *
 * The color is independent of the on/off state of the LED.
 *
 * @param index Index of the LED
 * @return Color of the LED
 */
template <size_t LED_COUNT>
Color WS2812Light<LED_COUNT>::getColor(uint32_t index) const {
    if (index >= LED_COUNT) {
        return Color(0, 0, 0);
    }
    return frame_buffer[index];
}
