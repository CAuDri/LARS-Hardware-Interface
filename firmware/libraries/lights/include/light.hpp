/**
 * @file light.hpp
 *
 * @brief CAuDri - Interface for controlling different types of lights
 *
 * This abstract base class defines the interface for various light types (e.g., LED strips, GPIO-controlled LEDs).
 * It includes methods for locking the light for exclusive access by a thread, as well as methods for turning lights on/off, setting colors, and brightness.
 *
 */
#pragma once

#include "cmsis_os2.h"
#include "colors.hpp"

/**
 * @brief Abstract base class for lights
 *
 * Derived classes should implement specific light types (e.g., WS2812 strip segments, GPIO-controlled LEDs, etc.)
 * A light can be locked for exclusive access by a thread using the lock() and unlock() methods.
 * All derived classes should check for lock ownership using isLockOwner() before performing operations.
 */
class Light {
   public:
    Light() = default;
    virtual ~Light() = default;

    bool lock();
    bool unlock();
    bool isLocked() const;

    virtual uint32_t getLEDCount() const = 0;

    // Generic light control functions
    virtual bool turnOn() = 0;
    virtual bool turnOff() = 0;
    virtual bool setColor(const Color& color, bool turn_on = true) = 0;
    virtual bool setBrightness(uint8_t brightness) = 0;

    virtual bool isOn() const = 0;
    virtual bool isOff() const = 0;
    virtual Color getColor() const = 0;

    // Indexed control functions for multi-LED strips
    virtual bool turnOn(uint32_t index) = 0;
    virtual bool turnOff(uint32_t index) = 0;
    virtual bool setColor(uint32_t index, const Color& color, bool turn_on = true) = 0;
    virtual bool setBrightness(uint32_t index, uint8_t brightness) = 0;

    virtual bool isOn(uint32_t index) const = 0;
    virtual bool isOff(uint32_t index) const = 0;
    virtual Color getColor(uint32_t index) const = 0;

   protected:
    osThreadId_t lock_owner = nullptr;

    bool isNotLockOwner() const;
};

/**
 * @brief Locks the light for exclusive access by the current thread
 *
 * @return true if the lock was acquired, false if already locked by another thread
 */
inline bool Light::lock() {
    osThreadId_t current_thread = osThreadGetId();
    if (lock_owner == nullptr) {
        lock_owner = current_thread;
        return true;
    }
    return lock_owner == current_thread;
}

/**
 * @brief Unlocks the light if owned by the current thread
 *
 * @return true if the lock was released, false if not owned by current thread
 */
inline bool Light::unlock() {
    osThreadId_t current_thread = osThreadGetId();
    if (lock_owner == current_thread) {
        lock_owner = nullptr;
        return true;
    }
    return false;
}

/**
 * @brief Checks if the light is currently locked
 *
 * @return true if locked, false otherwise
 */
inline bool Light::isLocked() const { return lock_owner != nullptr; }

/**
 * @brief Checks if the current thread is the lock owner
 *
 * @return false if current thread owns the lock, true otherwise
 */
inline bool Light::isNotLockOwner() const {
    if (lock_owner == nullptr) {
        return false;
    }
    osThreadId_t current_thread = osThreadGetId();
    return lock_owner != current_thread;
}
