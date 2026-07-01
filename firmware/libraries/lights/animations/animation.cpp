/**
 * @file animation.cpp
 *
 * @brief CAuDri - Interface for light animations
 *
 * Light animations will be used by the LightDispatcher to apply
 * various effects to multiple Light instances at once.
 *
 * Animations should be able to be cancelled at any time by setting the STOP_ANIMATION_FLAG.
 */
#include "animation.hpp"

/**
 * @brief Stop the animation
 *
 * This function signals the animation to stop by setting the STOP_ANIMATION_FLAG.
 * The actual stopping will occur in the context of the animation thread.
 *
 * @return true if the stop signal was sent successfully, false otherwise
 */
bool Animation::stop() {
    if (animation_thread == nullptr) {
        LogWarning("Animation: Stop called but no animation is running");
        return false;
    }
    osThreadFlagsSet(animation_thread, STOP_ANIMATION_FLAG);
    return true;
}

/**
 * @brief Helper function to exit animation on delay
 *
 * @param delay_ms Delay in milliseconds
 * @return true if stop flag was raised, false if delay completed without stop
 */
bool Animation::exitOnDelay(uint32_t delay_ms) {
    if (animation_thread != osThreadGetId()) {
        LogWarning("Animation: exitOnDelay called from wrong thread");
        return true;
    }
    auto flags = osThreadFlagsWait(STOP_ANIMATION_FLAG, osFlagsWaitAny, delay_ms);
    if (flags == osFlagsErrorTimeout) {
        return false;  // Normal timeout, no stop requested
    }
    if (!(flags & STOP_ANIMATION_FLAG)) {
        LogError("Animation: Unexpected error waiting for thread flags, flags: 0x%08lX", flags);
        return true;
    }
    // Stop flag was raised
    LogDebug("Animation: Stop flag received, exiting animation");
    return true;
}

/**
 * @brief Initialize and check animation parameters
 *
 * @return true if initialization successful, false otherwise
 */
bool Animation::init() {
    if (osThreadGetId() == nullptr) {
        LogError("Animation: Start called from non-thread context");
        return false;
    }
    if (animation_thread != nullptr) {
        LogWarning("Animation: Start called but an animation is already running");
        return false;
    }

    if (lights == nullptr || light_count == 0 || lights[0] == nullptr) {
        LogError("Animation: Start called but no lights are configured");
        return false;
    }
    if (light_count > light_array_size) {
        LogWarning("Animation: light_count (%zu) exceeds light_array size (%zu), reducing to fit", light_count, light_array_size);
        this->light_count = light_array_size;
    }

    // Check if all lights are actually valid
    for (size_t i = 0; i < this->light_count; ++i) {
        if (lights[i] == nullptr) {
            LogError("Animation: Light at index %zu is null, won't start animation", i);
            return false;
        }
    }

    animation_thread = osThreadGetId();
    osThreadFlagsClear(STOP_ANIMATION_FLAG | START_ANIMATION_FLAG);
    return true;
}