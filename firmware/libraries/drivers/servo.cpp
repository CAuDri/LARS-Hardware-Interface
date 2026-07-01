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
#include "thread_safe_adc.h"

#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

static constexpr uint32_t CONNECTION_CHECK_INITIAL_DELAY_MS = 200;  // Delay between starting the servo and the first feedback reading
static constexpr uint32_t CONNECTION_CHECK_START_ATTEMPTS = 5;  // Number of attempts to read valid feedback on startup
static constexpr uint32_t CONNECTION_CHECK_START_DELAY_MS = 100;  // Delay between attempts to read valid feedback on startup

static constexpr float CONNECTION_CHECK_ADC_EXTREMES_TOLERANCE = 0.1f;  // Tolerance as fraction of max ADC range
static constexpr float CONNECTION_CHECK_ANGLE_TOLERANCE_DEG = 5.0f;     // Tolerance in degrees for connection check

/**
 * @brief Construct a new Servo driver object for later initialization
 *
 * @param name The name of the servo driver instance
 */
Servo::Servo(const char* name) : Driver(name) {}

/**
 * @brief Construct a new Servo driver object with immediate initialization
 *
 * @param name The name of the servo driver instance
 * @param config Configuration parameters for the servo
 * @param calibration Calibration profile for the servo
 */
Servo::Servo(const char* name, const Config& config, Calibration& calibration) : Driver(name) {
    init(config, calibration);
}

Servo::~Servo() {
    // release();
}

/**
 * @brief Initialize the servo driver with the given configuration and calibration
 *
 * @param config Configuration parameters for the servo driver
 * @param calibration Calibration profile for the servo
 */
bool Servo::init(const Config& config, Calibration& calibration) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Cannot initialize, driver not uninitialized", getName());
        return false;
    }

    this->config = &config;

    if (!setCalibration(calibration)) {
        LogWarning("%s: Failed to set calibration values, using defaults", getName());
        this->calibration = Calibration();
    }

    if (!initTimerPWM()) {
        LogError("%s: Failed to initialize timer for PWM", getName());
        setState(State::ERROR);
        return false;
    }

    if (!initADC()) {
        LogError("%s: Failed to initialize ADC for feedback", getName());
        setState(State::ERROR);
        return false;
    }

    LogInfo("%s: Driver initialized", getName());
    setState(State::INITIALIZED);
    return true;
}

bool Servo::start() {
    if (state != State::INITIALIZED) {
        LogError("%s: Cannot start, driver not initialized", getName());
        return false;
    }
    setState(State::RUNNING);
    setConnectionState(ConnectionState::CONNECTING);

    if (!adc_initialized) {
        LogWarning("%s: ADC not initialized, skipping connection check", getName());
        setConnectionState(ConnectionState::UNKNOWN);
        LogInfo("%s: Driver started", getName());
        return true;
    }

    // Upon starting, set the servo to the center position
    setAngle(0.0f);
    osDelay(CONNECTION_CHECK_INITIAL_DELAY_MS);

    // Check for valid feedback value from the servo
    // This will update the connection state but we won't act on the result here
    for (uint32_t attempt = 0; attempt < CONNECTION_CHECK_START_ATTEMPTS; ++attempt) {
        if (checkConnection()) {
            break;
        }
        osDelay(CONNECTION_CHECK_START_DELAY_MS);
    }

    switch (connection_state) {
        case ConnectionState::CONNECTED:
            LogInfo("%s: Servo connection verified on start", getName());
            break;
        case ConnectionState::DISCONNECTED:
            LogWarning("%s: Servo connection failed on start", getName());
            break;
        case ConnectionState::CONNECTING:
            LogWarning("%s: Servo connection could not be determined on start", getName());
            break;
        default:
            break;
    }

    release();
    LogInfo("%s: Driver started", getName());
    return true;
}

/**
 * @brief Set the calibration profile for the servo
 *
 * @param calibration Calibration profile to set
 */
bool Servo::setCalibration(Calibration& calibration) {
    // TODO: Validate calibration values
    this->calibration = calibration;
    return true;
}

/**
 * @brief Initialize the timer for PWM generation
 *
 * @return true if initialization was successful, false otherwise
 */
bool Servo::initTimerPWM() {
    uint32_t timer_clock_freq = 0;
    TIM_HandleTypeDef* htim = config->htim;

    // Timers can be on different APB buses and thus have different clock frequencies
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 || htim->Instance == TIM10 ||
        htim->Instance == TIM11) {
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

/**
 * @brief Initialize the ADC for feedback reading
 *
 * @return true if initialization was successful, false otherwise
 */
bool Servo::initADC() {
    if (config->hadc == nullptr) {
        LogWarning("%s: No ADC handle provided, skipping ADC initialization", getName());
        return true;
    }

    // Register the ADC channel with the thread-safe ADC wrapper
    if (ADC_RegisterChannel(config->hadc, config->adc_channel) != HAL_OK) {
        LogError("%s: Failed to register ADC channel %lu", getName(), config->adc_channel);
        return false;
    }

    LogDebug("%s: ADC initialized for feedback on channel %lu", getName(), config->adc_channel);
    adc_initialized = true;
    return true;
}

/**
 * @brief Set the servo to the specified angle in degrees
 *
 * @param angle_deg Desired angle in degrees
 * @return true if the angle was set successfully, false otherwise
 */
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

    last_set_pulse_width_us = pulse_width;
    last_set_angle_deg = angle_deg;

    LogVerbose("%s: Set angle to %.2f deg (Pulse width: %lu us)", getName(), angle_deg, pulse_width);
    return true;
}

/**
 * @brief Set the servo to the specified pulse width in microseconds
 *
 * @param pulse_width_us Desired pulse width in microseconds
 * @return true if the pulse width was set successfully, false otherwise
 */
bool Servo::setPulseWidth(uint32_t pulse_width_us) {
    if (state != State::RUNNING) {
        LogWarning("%s: Cannot set pulse width, driver not running", getName());
        return false;
    }

    pulse_width_us = std::clamp(pulse_width_us, calibration.min_pulse_us, calibration.max_pulse_us);

    __HAL_TIM_SET_COMPARE(config->htim, config->tim_channel, pulse_width_us);

    last_set_pulse_width_us = pulse_width_us;

    LogVerbose("%s: Set pulse width to %lu us", getName(), pulse_width_us);
    return true;
}

/**
 * @brief Get the current angle of the servo in degrees
 *
 * @param angle_deg Reference to store the current angle in degrees
 * @return true if the angle was retrieved successfully, false otherwise
 */
bool Servo::getAngle(float& angle_deg) const {
    uint16_t adc_value = 0;
    if (!getADCValue(adc_value)) {
        return false;
    }

    angle_deg = adcToAngle(adc_value);
    return true;
}

/**
 * @brief Get the current ADC feedback value
 *
 * @param adc_value Reference to store the ADC value
 * @return true if the ADC value was retrieved successfully, false otherwise
 */
bool Servo::getADCValue(uint16_t& adc_value) const {
    if (state != State::RUNNING || !adc_initialized) {
        LogWarning("%s: Cannot get ADC value, driver not running or ADC not initialized", getName());
        return false;
    }

    HAL_StatusTypeDef status = ADC_GetValue(config->hadc, config->adc_channel, &adc_value);
    if (status != HAL_OK) {
        LogWarning("%s: Cannot get ADC value, ADC_GetValue failed with status %d", getName(), status);
        return false;
    }
    return true;
}

/**
 * @brief Release the servo (rotating freely) by stopping PWM generation
 *
 * @return true if the servo was released successfully, false otherwise
 */
bool Servo::release() {
    if (state != State::RUNNING && state != State::INITIALIZED) {
        return false;
    }

    // Stop PWM generation by setting pulse width to 0
    __HAL_TIM_SET_COMPARE(config->htim, config->tim_channel, 0);
    return true;
}

/**
 * @brief Check and update the connection state of the servo
 *
 * For a servo with analog feedback, we consider it connected if:
 * - The ADC feedback value is within the calibrated range (with some tolerance)
 * - The reported angle is close to the last set angle (within a tolerance)
 *
 * This method should only be called after the servo *angle* has been set
 * and some time has passed for the servo to respond.
 *
 * @return true if the servo is connected, false otherwise
 */
bool Servo::checkConnection() {
    if (state != State::RUNNING) {
        LogWarning("%s: Cannot check connection, driver not running", getName());
        return false;
    }
    if (!adc_initialized) {
        LogWarning("%s: Cannot check connection, ADC not initialized", getName());
        connection_state = ConnectionState::UNKNOWN;
        return false;
    }

    // For a servo with analog feedback, we can check the ADC value to determine connection
    uint16_t adc_value = 0;
    if (!getADCValue(adc_value)) {
        connection_state = ConnectionState::DISCONNECTED;
        return false;
    }

    // Check if the ADC value is within the calibrated feedback range
    uint16_t max_adc = (1 + CONNECTION_CHECK_ADC_EXTREMES_TOLERANCE) * calibration.max_feedback;
    uint16_t min_adc = (1 - CONNECTION_CHECK_ADC_EXTREMES_TOLERANCE) * calibration.min_feedback;
    if (adc_value < min_adc || adc_value > max_adc) {
        connection_state = ConnectionState::DISCONNECTED;
        return false;
    }

    // Check if the angle is close to the set angle (within tolerance)
    float current_angle = adcToAngle(adc_value);
    if (std::abs(current_angle - last_set_angle_deg) > CONNECTION_CHECK_ANGLE_TOLERANCE_DEG) {
        connection_state = ConnectionState::DISCONNECTED;
        return false;
    }

    connection_state = ConnectionState::CONNECTED;
    return true;
}

/**
 * @brief Convert an angle in degrees to the corresponding pulse width in microseconds
 *
 * @param angle_deg Angle in degrees
 * @return Corresponding pulse width in microseconds
 */
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
        pulse_width_us = calibration.center_pulse_us -
                         (calibration.center_pulse_us - calibration.min_pulse_us) * angle_deg / calibration.min_angle;
    } else {
        // Map from [0, max_angle] to [center_pulse, max_pulse]
        pulse_width_us = calibration.center_pulse_us +
                         (calibration.max_pulse_us - calibration.center_pulse_us) * angle_deg / calibration.max_angle;
    }

    return pulse_width_us;
}

/**
 * @brief Convert an ADC feedback value to the corresponding angle in degrees
 *
 * @param adc_value ADC feedback value
 * @return Corresponding angle in degrees
 */
float Servo::adcToAngle(uint32_t adc_value) const {
    // Map the ADC value to the angle range:
    //      ADC = min_feedback -> angle = min_angle
    //      ADC = max_feedback -> angle = max_angle
    float angle_deg = calibration.min_angle + (calibration.max_angle - calibration.min_angle) *
                                                  (static_cast<float>(adc_value - calibration.min_feedback) /
                                                   static_cast<float>(calibration.max_feedback - calibration.min_feedback));

    // Invert the angle if necessary
    if (config->inverted) {
        angle_deg = -angle_deg;
    }

    return angle_deg;
}
