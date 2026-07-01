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

#include <array>

#include "cmsis_os2.h"
#include "light.hpp"
#include "logger.h"

constexpr uint32_t START_ANIMATION_FLAG = 0x400;  // Thread flag to start the animation (change on conflicts)
constexpr uint32_t STOP_ANIMATION_FLAG = 0x800;   // Thread flag to stop the animation (change on conflicts)

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
inline bool Animation::start(Light* (&light_array)[N], uint32_t light_count) {
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
inline bool Animation::start(std::array<Light*, N>& light_array, uint32_t light_count) {
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