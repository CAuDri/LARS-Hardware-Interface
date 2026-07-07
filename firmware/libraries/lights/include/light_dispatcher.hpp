/**
 * @file light_dispatcher.hpp
 *
 * @brief CAuDri - Light dispatcher for managing multiple lights
 *
 * Multiple Lights can be registered with the LightDispatcher to control them together.
 * Different animations can be applied which will be reflected on all registered lights.
 * They will run in a dedicated background thread without blocking the caller.
 */
#pragma once

#include "light.hpp"
#include "animation.hpp"
#include "cmsis_os2.h"

#define LIGHT_DISPATCHER_MAX_LIGHTS 4

#define LIGHT_DISPATCHER_THREAD_STACK_SIZE 1024
#define LIGHT_DISPATCHER_THREAD_PRIORITY osPriorityLow

class LightDispatcher {
public:
    LightDispatcher(const char* name = "Light Dispatcher");
    ~LightDispatcher();

    bool start();

    bool registerLight(Light& light, Color default_color = COLOR_OFF);
    bool unregisterLight(Light& light);

    bool turnOn(Color color_override = COLOR_OFF);
    bool turnOff();

    bool runAnimation(Animation* animation);
    bool abortAnimation();
    Animation* getCurrentAnimation();
    bool isAnimationRunning();

    bool lockLights();
    bool unlockLights();

private:
    const char* dispatcher_name;

    Light* lights[LIGHT_DISPATCHER_MAX_LIGHTS];
    Color default_colors[LIGHT_DISPATCHER_MAX_LIGHTS];
    size_t light_count = 0;

    Animation* current_animation = nullptr;
    Animation* pending_animation = nullptr;

    osThreadId_t dispatcher_thread_id = nullptr;
    osThreadAttr_t dispatcher_thread_attributes;
    StaticTask_t dispatcher_thread_control_block;
    uint8_t dispatcher_thread_stack[LIGHT_DISPATCHER_THREAD_STACK_SIZE];

    void dispatcherThread();
};
