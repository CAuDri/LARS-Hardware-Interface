/**
 * @file light_dispatcher.cpp
 *
 * @brief CAuDri - Light dispatcher for managing multiple lights
 *
 * Multiple Lights can be registered with the LightDispatcher to control them together.
 * Different animations can be applied which will be reflected on all registered lights.
 * They will run in a dedicated background thread without blocking the caller.
 */
#include "light_dispatcher.hpp"

/**
 * @brief Construct a new Light Dispatcher object
 */
LightDispatcher::LightDispatcher(const char* name) : dispatcher_name(name) {
    dispatcher_thread_attributes.name = dispatcher_name;
    dispatcher_thread_attributes.priority = LIGHT_DISPATCHER_THREAD_PRIORITY;
    dispatcher_thread_attributes.stack_mem = &dispatcher_thread_stack;
    dispatcher_thread_attributes.stack_size = sizeof(dispatcher_thread_stack);
    dispatcher_thread_attributes.cb_mem = &dispatcher_thread_control_block;
    dispatcher_thread_attributes.cb_size = sizeof(dispatcher_thread_control_block);

    dispatcher_thread_id = osThreadNew(
        // Helper function for using a non-static method as the thread entry point
        // The 'this' pointer is passed as the user argument to the lambda
        [](void* arg) -> void {
            auto* obj = static_cast<LightDispatcher*>(arg);
            obj->dispatcherThread();
        },
        this,
        &dispatcher_thread_attributes);

    if (dispatcher_thread_id == nullptr) {
        LogError("Light Dispatcher: Failed to create dispatcher thread");
    } else {
        LogDebug("Light Dispatcher: Dispatcher thread created");
    }
}

/**
 * @brief Destroy the Light Dispatcher object
 */
LightDispatcher::~LightDispatcher() { osThreadTerminate(dispatcher_thread_id); }

/**
 * @brief Register a Light instance with the dispatcher
 *
 * The Light will be controlled by the dispatcher and included in animations.
 *
 * @param light Reference to the Light instance to register
 * @param default_color Default color to set when turning on the light
 * @return true if the Light was successfully registered
 */
bool LightDispatcher::registerLight(Light& light, Color default_color) {
    if (light_count >= LIGHT_DISPATCHER_MAX_LIGHTS) {
        return false;
    }
    lights[light_count] = &light;
    default_colors[light_count] = default_color;
    light_count++;

    LogDebug("Light Dispatcher: Registered light, total count: %d", light_count);
    return true;
}

/**
 * @brief Unregister a Light instance from the dispatcher
 *
 * The Light will no longer be controlled by the dispatcher.
 *
 * @param light Reference to the Light instance to unregister
 * @return true if the Light was successfully unregistered
 */
bool LightDispatcher::unregisterLight(Light& light) {
    for (size_t i = 0; i < light_count; i++) {
        if (lights[i] == &light) {
            // Shift remaining lights down
            for (size_t j = i; j < light_count - 1; j++) {
                lights[j] = lights[j + 1];
                default_colors[j] = default_colors[j + 1];
            }
            light_count--;

            LogDebug("Light Dispatcher: Unregistered light, total count: %d", light_count);
            return true;
        }
    }
    LogError("Light Dispatcher: Failed to unregister light, not found");
    return false;
}

/**
 * @brief Helper function to turn on all registered lights
 *
 * @param color_override Optional color to set all lights to when turning on. If COLOR_OFF, use each light's default color.
 * @return true if all lights were successfully turned on
 */
bool LightDispatcher::turnOn(Color color_override) {
    if (light_count == 0) {
        LogWarning("Light Dispatcher: No lights registered to turn on");
        return false;
    }
    if (isAnimationRunning()) {
        if (!abortAnimation()) {
            return false;
        }
        // Ensure the dispatcher thread has time to abort the animation
        // On high system load, this delay might not be sufficient - in that case, consider a better synchronization mechanism
        osDelay(1);
    }

    for (size_t i = 0; i < light_count; i++) {
        Color color_to_set = (color_override != COLOR_OFF) ? color_override : default_colors[i];
        if (!lights[i]->setColor(color_to_set, true)) {
            LogWarning("Light Dispatcher: Failed to turn on light %d", i);
            return false;
        }
    }
    return true;
}

/**
 * @brief Helper function to turn off all registered lights
 *
 * @return true if all lights were successfully turned off
 */
bool LightDispatcher::turnOff() {
    if (light_count == 0) {
        LogWarning("Light Dispatcher: No lights registered to turn off");
        return false;
    }
    if (isAnimationRunning()) {
        if (!abortAnimation()) {
            return false;
        }
        // Ensure the dispatcher thread has time to abort the animation
        // On high system load, this delay might not be sufficient - in that case, consider a better synchronization mechanism
        osDelay(1);
    }

    for (size_t i = 0; i < light_count; i++) {
        if (!lights[i]->turnOff()) {
            LogWarning("Light Dispatcher: Failed to turn off light %d", i);
            return false;
        }
    }
    return true;
}

/**
 * @brief Run an animation on all registered lights
 *
 * The animation will be started in the background dispatcher thread.
 * If another animation is currently running, it will be aborted first.
 *
 * @param animation Reference to the Animation instance to run
 * @return true if the animation was successfully started
 */
bool LightDispatcher::runAnimation(Animation* animation) {
    if (light_count == 0) {
        LogWarning("Light Dispatcher: No lights registered to run animation");
        return false;
    }
    if (isAnimationRunning()) {
        if (!abortAnimation()) {
            return false;
        }
    }

    pending_animation = animation;
    osThreadFlagsSet(dispatcher_thread_id, START_ANIMATION_FLAG);
    return true;
}

/**
 * @brief Abort the currently running animation
 *
 * @return true if the animation was successfully aborted or no animation was running
 */
bool LightDispatcher::abortAnimation() {
    if (current_animation == nullptr) {
        return true;  // No animation running
    }

    // This will set the ANIMATION_STOP_FLAG for the dispatcher thread stored in the Animation instance
    // This will be done asynchronously, so this function can return immediately
    if (!current_animation->stop()) {
        LogError("Light Dispatcher: Failed to abort current animation");
        return false;
    }

    return true;
}

/**
 * @brief Get the currently running animation
 *
 * @return Pointer to the current Animation instance, or nullptr if none is running
 */
Animation* LightDispatcher::getCurrentAnimation() { return current_animation; }

/**
 * @brief Check if an animation is currently running
 *
 * @return true if an animation is running, false otherwise
 */
bool LightDispatcher::isAnimationRunning() { return current_animation != nullptr; }

/**
 * @brief Lock all registered lights for exclusive access by the dispatcher
 *
 * @return true if all lights were successfully locked
 */
bool LightDispatcher::lockLights() {
    bool ret = true;
    for (size_t i = 0; i < light_count; i++) {
        if (!lights[i]->lock(dispatcher_thread_id)) {
            LogWarning("Light Dispatcher: Failed to lock light %d", i);
            ret = false;
        }
    }
    return ret;
}

/**
 * @brief Unlock all registered lights
 *
 * @return true if all lights were successfully unlocked
 */
bool LightDispatcher::unlockLights() {
    bool ret = true;
    for (size_t i = 0; i < light_count; i++) {
        if (!lights[i]->unlock(dispatcher_thread_id)) {
            LogWarning("Light Dispatcher: Failed to unlock light %d", i);
            ret = false;
        }
    }
    return ret;
}

/**
 * @brief Dispatcher thread function
 *
 * This function runs in a dedicated thread and handles animations and light updates.
 */
void LightDispatcher::dispatcherThread() {
    LogDebug("Light Dispatcher: Dispatcher thread started");

    while (true) {
        // Wait for the caller to start an animation or control the lights
        auto flags = osThreadFlagsWait(START_ANIMATION_FLAG, osFlagsWaitAny, osWaitForever);
        if (flags & osFlagsError) {
            LogError("Light Dispatcher: Error waiting for thread flags, flags: 0x%08lX", flags);
            osDelay(100);  // Artificial delay to prevent tight error loop
            continue;
        }

        // No animation should currently be running or we would not have gotten here
        if (current_animation != nullptr) {
            LogError("Light Dispatcher: Animation already running, cannot start a new one");
            continue;
        }
        // Check if there is a pending animation to run
        if (pending_animation == nullptr) {
            LogError("Light Dispatcher: No pending animation to run");
            continue;
        }

        current_animation = pending_animation;
        pending_animation = nullptr;

        LogDebug("Light Dispatcher: Starting new animation");

        // Run the animation until completion or the STOP flag is raised
        bool completed = current_animation->start(lights, light_count);
        if (!completed) {
            LogDebug("Light Dispatcher: Current animation was aborted");
        } else {
            LogDebug("Light Dispatcher: Current animation completed");
        }

        current_animation = nullptr;
    }
}
