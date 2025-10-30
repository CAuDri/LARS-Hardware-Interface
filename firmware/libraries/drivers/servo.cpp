/**
 * @file servo.cpp
 *
 * @brief CAuDri - PWM based Servo Driver with analog feedback
 *
 * This driver allows control of standard PWM servos and reads their position via an analog feedback.
 * A calibration profile can be provided to map PWM and feedback values to meaningful angles.
 */
#include "servo.hpp"

#include <algorithm>

#include "logger.h"

Servo::Servo(const char* name) : Driver(name) {}

Servo::Servo(const char* name, const Config& config, Calibration& calibration) : Driver(name) {
    init(config, calibration);
}

Servo::~Servo() {
    // release();
}

bool Servo::init(const Config& config, Calibration& calibration) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Cannot initialize, driver not uninitialized", getName());
        return false;
    }

    this->config = &config;

    if (!setCalibration(calibration)) {
        LogWarning("%s: Failed to set calibration values, using defaults", getName());
    }

    if (!initTimerPWM()) {
        LogError("%s: Failed to initialize timer for PWM", getName());
        setState(State::ERROR);
        return false;
    }

    // if (!initADC()) {
    //     LogError("%s: Failed to initialize ADC for feedback", getName());
    //     setState(State::ERROR);
    //     return false;
    // }

    LogInfo("%s: Driver initialized", getName());
    setState(State::INITIALIZED);
    setState(State::RUNNING);
    return true;
}

bool Servo::setCalibration(Calibration& calibration) {
    // TODO: Validate calibration values
    this->calibration = calibration;
    return true;
}

bool Servo::initTimerPWM() {
    uint32_t timer_clock_freq = 0;
    TIM_HandleTypeDef* htim = config->htim;

    // Timers can be on different APB buses and thus have different clock frequencies
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 ||
        htim->Instance == TIM10 || htim->Instance == TIM11) {
        // TIM1, TIM8, TIM9, TIM10, TIM11 are on APB2
        // This might only be valid for STM32F4xx and needs to be adapted for other STM32 series
        timer_clock_freq = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            // Timer clock is automatically doubled if APB prescaler > 1
            timer_clock_freq *= 2;
        }
    } else {
        // All other timers are on APB1
        timer_clock_freq = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
            timer_clock_freq *= 2;
        }
    }

    // Scale the timer clock frequency for a 1 us resolution
    uint32_t prescaler = (timer_clock_freq / 1e6) - 1;

    // Check if the prescaler value is within the allowed range
    if (prescaler > 0xFFFF || prescaler == 0) {
        LogError("%s: Invalid prescaler value: %lu", getName(), prescaler);
        return false;
    }

    // Set timer prescaler and auto-reload for 20 ms period
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(htim, SERVO_PWM_PERIOD_US - 1);

    // Reset the counter to avoid any glitches
    __HAL_TIM_SET_COUNTER(htim, 0);

    // Make sure no PWM signal is generated when starting the timer
    __HAL_TIM_SET_COMPARE(htim, config->tim_channel, 0);

    // Start the PWM generation
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim, config->tim_channel);
    if (status != HAL_OK) {
        LogError("%s: HAL_TIM_PWM_Start failed with status %d", getName(), status);
        return false;
    }

    LogDebug("%s: Timer PWM initialized (Prescaler: %lu, ARR: %lu)", getName(), prescaler, SERVO_PWM_PERIOD_US - 1);
    return true;
}

bool Servo::setAngle(float angle_deg) {
    if (state != State::RUNNING) {
        LogWarning("%s: Cannot set angle, driver not running", getName());
        return false;
    }

    uint32_t pulse_width = angleToPulseWidth(angle_deg);
    pulse_width = std::clamp(pulse_width, calibration.min_pulse_us, calibration.max_pulse_us);

    // Each tick of the timer corresponds to 1 us due to our prescaler setting
    // We can directly set the compare register to the desired pulse width
    __HAL_TIM_SET_COMPARE(config->htim, config->tim_channel, pulse_width);

    LogDebug("%s: Set angle to %.2f deg (Pulse width: %lu us)", getName(), angle_deg, pulse_width);
    return true;
}

bool Servo::setPulseWidth(uint32_t pulse_width_us) {
    if (state != State::RUNNING) {
        LogWarning("%s: Cannot set pulse width, driver not running", getName());
        return false;
    }

    pulse_width_us = std::clamp(pulse_width_us, calibration.min_pulse_us, calibration.max_pulse_us);

    __HAL_TIM_SET_COMPARE(config->htim, config->tim_channel, pulse_width_us);

    LogDebug("%s: Set pulse width to %lu us", getName(), pulse_width_us);
    return true;
}

uint32_t Servo::angleToPulseWidth(float angle_deg) const {
    // We will invert the angle before doing any calculations
    // This way we don't have to change any calibration values
    if (config->inverted) {
        angle_deg = -angle_deg;
    }

    // Clamp the angle to the calibrated range
    angle_deg = std::clamp(angle_deg, calibration.min_angle, calibration.max_angle);

    // Map the angle to the pulse width range:
    //      Angle = angle_min -> pulse_width = pulse_min
    //      Angle = 0°        -> pulse_width = pulse_mid
    //      Angle = angle_max -> pulse_width = pulse_max
    // For each steering direction, we assume a linear relationship between angle and pulse width
    uint32_t pulse_width_us = 0;
    if (angle_deg < 0) {
        // Map from [min_angle, 0] to [min_pulse, center_pulse]
        pulse_width_us = calibration.center_pulse_us - (calibration.center_pulse_us - calibration.min_pulse_us) *
                                                           angle_deg / calibration.min_angle;
    } else {
        // Map from [0, max_angle] to [center_pulse, max_pulse]
        pulse_width_us = calibration.center_pulse_us + (calibration.max_pulse_us - calibration.center_pulse_us) *
                                                           angle_deg / calibration.max_angle;
    }

    return pulse_width_us;
}
