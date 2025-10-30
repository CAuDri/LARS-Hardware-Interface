/**
 * @file servo.hpp
 *
 * @brief CAuDri - PWM based Servo Driver with analog feedback
 *
 * This driver allows control of standard PWM servos and reads their position via an analog feedback
 * pin. A calibration profile can be provided to map PWM and feedback values to meaningful angles.
 */
#pragma once

#include "driver.hpp"
#include "stm32f4xx_hal.h"

// Default values for the servo calibration
constexpr float SERVO_DEFAULT_MIN_ANGLE_DEG = -90.0f;
constexpr float SERVO_DEFAULT_MAX_ANGLE_DEG = 90.0f;
constexpr uint32_t SERVO_DEFAULT_MIN_PULSE_US = 1000;
constexpr uint32_t SERVO_DEFAULT_CENTER_PULSE_US = 1500;
constexpr uint32_t SERVO_DEFAULT_MAX_PULSE_US = 2000;
constexpr uint32_t SERVO_DEFAULT_MIN_FEEDBACK = 0;
constexpr uint32_t SERVO_DEFAULT_MAX_FEEDBACK = 4095;

// Standard servo PWM period of 20 ms
constexpr uint32_t SERVO_PWM_PERIOD_US = 20000;  

class Servo : public Driver {
   public:
    struct Config {
        TIM_HandleTypeDef* htim = nullptr;  // Timer handle for PWM generation
        uint32_t tim_channel = 0;           // Timer channel for PWM output
        ADC_HandleTypeDef* hadc = nullptr;  // ADC handle for feedback reading
        uint32_t adc_channel = 0;           // ADC channel for feedback
        bool inverted = false;              // Whether the servo direction is inverted
    };

    struct Calibration {
        float min_angle = SERVO_DEFAULT_MIN_ANGLE_DEG;
        float max_angle = SERVO_DEFAULT_MAX_ANGLE_DEG;
        uint32_t min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;  // Minimum pulse width in microseconds
        uint32_t center_pulse_us = SERVO_DEFAULT_CENTER_PULSE_US;  // Center pulse width in microseconds
        uint32_t max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;  // Maximum pulse width in microseconds
        uint32_t min_feedback = SERVO_DEFAULT_MIN_FEEDBACK;  // Minimum ADC feedback value
        uint32_t max_feedback = SERVO_DEFAULT_MAX_FEEDBACK;  // Maximum ADC feedback value
    };

    Servo(const char* name);
    explicit Servo(const char* name, const Config& config, Calibration& calibration);
    ~Servo() override;

    bool init(const Config& config, Calibration& calibration);
    bool setCalibration(Calibration& calibration);
    bool start();

    bool setAngle(float angle_deg);
    bool setPulseWidth(uint32_t pulse_width_us);
    float getAngle() const;
    float getADCValue() const;

    bool release();

   private:
    const Config* config = nullptr;
    Calibration calibration;

    bool initTimerPWM();
    bool initADC();

    uint32_t angleToPulseWidth(float angle_deg) const;
    float feedbackToAngle(uint32_t adc_value) const;
};