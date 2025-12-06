/**
 * @file animation.hpp
 *
 * @brief CAuDri - Interface for light animations
 *
 * Light animations will be used by the LightDispatcher to apply
 * various effects to multiple Light instances at once.
 *
 * Animations should be able to be cancelled at any time by setting the STOP_ANIMATION_FLAG.
 */
#pragma once

#include "light.hpp"
#include "logger.h"

constexpr uint32_t STOP_ANIMATION_FLAG = 0x800;  // Thread flag to stop the animation (change on conflicts)

/**
 * @brief Base class for light animations
 *
 * Animations should be derived from this class and implement the run() method.
 */
class Animation {
   public:
    Animation() = default;
    virtual ~Animation() = default;

    template <size_t N>
    bool start(Light* (&light_array)[N], uint32_t light_count);

    template <size_t N>
    bool start(std::array<Light*, N>& light_array, uint32_t light_count);

    bool stop();

   protected:
    Light** lights;
    size_t light_array_size;  // Size of the light pointer array

    size_t light_count;  // Number of actual light objects in the array

    osThreadId_t animation_thread = nullptr;

    /**
     * @brief Run the animation
     *
     * This function should be implemented by derived classes handling all of the animation logic. Delays and exit behavior need to be handled by the implementation itself.
     * exitOnDelay() can be used as a helper for exit conditions based on time delays.
     *
     * @return true if the animation completed successfully, false if it was stopped or encountered an error
     */
    virtual bool run() = 0;

    bool exitOnDelay(uint32_t delay_ms);

   private:
    bool init();
};

/**
 * @brief Set up and start the animation
 *
 * This function will run in the context of the calling thread.
 * It will only return once the animation has completed or been stopped midway.
 *
 * @param light_array Array of Light pointers to apply the animation to
 *
 * @return true if animation completed successfully, false otherwise
 */
template <size_t N>
bool Animation::start(Light* (&light_array)[N], uint32_t light_count) {
    this->lights = light_array;
    this->light_array_size = N;
    this->light_count = light_count;

    if (!init()) {
        return false;
    }

    // Run the actual animation
    bool completed = run();

    animation_thread = nullptr;
    return completed;
}

/**
 * @brief Set up and start the animation
 *
 * This function will run in the context of the calling thread.
 * It will only return once the animation has completed or been stopped midway.
 *
 * @param light_array Array of Light pointers to apply the animation to
 *
 * @return true if animation completed successfully, false otherwise
 */
template <size_t N>
bool Animation::start(std::array<Light*, N>& light_array, uint32_t light_count) {
    this->lights = light_array.data();
    this->light_array_size = N;
    this->light_count = light_count;

    if (!init()) {
        return false;
    }

    // Run the actual animation
    bool completed = run();

    animation_thread = nullptr;
    return completed;
}

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
    osThreadFlagsClear(STOP_ANIMATION_FLAG);
    return true;
}