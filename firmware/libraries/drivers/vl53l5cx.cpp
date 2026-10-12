/**
 * @file vl53l5cx.cpp
 *
 * @brief ST VL53L5CX multizone Time-of-Flight driver implementation
 */
#include "vl53l5cx.hpp"

#include <cstring>

#include "logger.h"
#include "thread_safe_gpio.h"

extern "C" {
uint8_t vl53l5cx_calibrate_xtalk(VL53L5CX_Configuration* device, uint16_t reflectance_percent, uint8_t sample_count, uint16_t distance_mm);
uint8_t vl53l5cx_get_caldata_xtalk(VL53L5CX_Configuration* device, uint8_t* data);
uint8_t vl53l5cx_set_caldata_xtalk(VL53L5CX_Configuration* device, uint8_t* data);
uint8_t vl53l5cx_get_xtalk_margin(VL53L5CX_Configuration* device, uint32_t* margin);
uint8_t vl53l5cx_set_xtalk_margin(VL53L5CX_Configuration* device, uint32_t margin);
}

static constexpr uint32_t VL53L5CX_LPN_BOOT_DELAY_MS = 100;
static constexpr uint32_t VL53L5CX_LPN_RESET_DELAY_MS = 10;
static constexpr uint8_t VL53L5CX_TARGET_STATUS_VALID = 5;
static constexpr uint8_t VL53L5CX_TARGET_STATUS_VALID_LARGE_PULSE = 9;

static_assert(sizeof(VL53L5CX_Configuration) <= 2400U,
              "VL53L5CX output selection unexpectedly increased the ULD context");
static_assert(sizeof(VL53L5CX_ResultsData) <= 324U, "VL53L5CX output selection unexpectedly increased ranging results");

// A ULD context contains its 1 KiB transfer scratch buffer, NVM/offset data,
// and crosstalk data. Reserve all four contexts in CCMRAM and assign them only
// to initialized driver instances so unused application objects remain small.
static VL53L5CX_Configuration vl53l5cx_device_pool[VL53L5CX_MAX_INSTANCES]
    __attribute__((section(".ccmram.vl53l5cx"), aligned(4)));
static bool vl53l5cx_device_slots[VL53L5CX_MAX_INSTANCES]{};
static I2C_HandleTypeDef* vl53l5cx_device_buses[VL53L5CX_MAX_INSTANCES]{};
static uint8_t vl53l5cx_device_addresses[VL53L5CX_MAX_INSTANCES]{};
static GPIO_TypeDef* vl53l5cx_lpn_ports[VL53L5CX_MAX_INSTANCES]{};
static uint16_t vl53l5cx_lpn_pins[VL53L5CX_MAX_INSTANCES]{};
static bool vl53l5cx_device_enabled[VL53L5CX_MAX_INSTANCES]{};

static bool isSingleGpioPin(uint16_t pin) { return pin != 0U && pin <= GPIO_PIN_15 && (pin & (pin - 1U)) == 0U; }

/**
 * @brief Construct an uninitialized VL53L5CX driver.
 * @param name Stable driver name used in logs and diagnostics.
 */
VL53L5CX::VL53L5CX(const char* name) : TofDriver(name) {}

/**
 * @brief Construct and initialize a VL53L5CX driver.
 * @param name Stable driver name used in logs and diagnostics.
 * @param config Persistent hardware and ranging configuration.
 * @param calibration Optional persistent crosstalk calibration.
 */
VL53L5CX::VL53L5CX(const char* name, const Config& config, const Calibration* calibration) : TofDriver(name) {
    init(config, calibration);
}

/**
 * @brief Release the EXTI registration and shared ULD context.
 */
VL53L5CX::~VL53L5CX() {
    releaseInterrupt();
    releaseDeviceContext();
}

/**
 * @brief Validate and register the sensor's static resources.
 *
 * Initialization keeps the sensor disabled through LPn. Firmware upload,
 * address assignment, and ranging configuration are deferred to start().
 *
 * @param config Persistent configuration that must remain unchanged.
 * @param calibration Optional persistent crosstalk calibration.
 * @return true when the driver is ready to start.
 */
bool VL53L5CX::init(const Config& config, const Calibration* calibration) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Cannot initialize, driver not uninitialized", getName());
        return false;
    }
    if (!validateConfig(config)) {
        setState(State::ERROR);
        return false;
    }

    this->config = &config;
    this->calibration = calibration;

    for (size_t i = 0; i < VL53L5CX_MAX_INSTANCES; ++i) {
        if (vl53l5cx_device_slots[i] && vl53l5cx_device_buses[i] == config.hi2c &&
            vl53l5cx_device_addresses[i] == config.address) {
            LogError("%s: VL53L5CX address 0x%02X is already registered", getName(), config.address);
            setState(State::ERROR);
            return false;
        }
        if (vl53l5cx_device_slots[i] && vl53l5cx_lpn_ports[i] == config.lpn_port && vl53l5cx_lpn_pins[i] == config.lpn_pin) {
            LogError("%s: VL53L5CX LPn GPIO is already registered", getName());
            setState(State::ERROR);
            return false;
        }
    }

    if (!acquireDeviceContext()) {
        LogError("%s: No VL53L5CX device context available (maximum %u)", getName(), static_cast<unsigned>(VL53L5CX_MAX_INSTANCES));
        setState(State::ERROR);
        return false;
    }
    if (!initMutex()) {
        LogError("%s: Failed to create VL53L5CX mutex", getName());
        releaseDeviceContext();
        setState(State::ERROR);
        return false;
    }
    if (!initGpios()) {
        releaseInterrupt();
        releaseDeviceContext();
        setState(State::ERROR);
        return false;
    }

    std::memset(device, 0, sizeof(*device));
    device->platform.hi2c = config.hi2c;
    device->platform.address = static_cast<uint16_t>(VL53L5CX_DEFAULT_ADDRESS) << 1U;

    setState(State::INITIALIZED);
    setConnectionState(ConnectionState::DISCONNECTED);
    LogInfo("%s: VL53L5CX driver initialized for address 0x%02X", getName(), config.address);
    return true;
}

/**
 * @brief Enable the sensor, upload its firmware if needed, and start ranging.
 * @return true when the sensor entered the configured ranging mode.
 */
bool VL53L5CX::start() {
    if (state != State::INITIALIZED) {
        LogError("%s: Cannot start VL53L5CX, driver not initialized", getName());
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire VL53L5CX mutex while starting", getName());
        return false;
    }

    setConnectionState(ConnectionState::CONNECTING);
    const bool success = (sensor_initialized || initializeSensor()) && configureSensor(true);
    if (success) {
        setState(State::RUNNING);
        setConnectionState(ConnectionState::CONNECTED);
        consecutive_read_failures = 0;
        LogInfo("%s: VL53L5CX started at 0x%02X", getName(), config->address);
    } else {
        (void)setEnabled(false);
        setConnectionState(ConnectionState::DISCONNECTED);
        setState(State::ERROR);
        LogError("%s: Failed to start VL53L5CX", getName());
    }

    unlock();
    return success;
}

/**
 * @brief Stop ranging while retaining the uploaded firmware and configuration.
 * @return true when the ULD confirmed that ranging stopped.
 */
bool VL53L5CX::stop() {
    if (state != State::RUNNING || device == nullptr) {
        LogError("%s: Cannot stop VL53L5CX, driver not running", getName());
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire VL53L5CX mutex while stopping", getName());
        return false;
    }

    const uint8_t status = vl53l5cx_stop_ranging(device);
    if (status == VL53L5CX_STATUS_OK) {
        interrupt_pending = false;
        read_retry_pending = false;
        setState(State::INITIALIZED);
        setConnectionState(ConnectionState::CONNECTED);
    } else {
        logApiError("stop ranging", status);
        setConnectionState(ConnectionState::DISCONNECTED);
    }

    unlock();
    return status == VL53L5CX_STATUS_OK;
}

/**
 * @brief Reinitialize and reconfigure the sensor.
 *
 * A previously running driver resumes ranging. A stopped or failed driver is
 * returned to the initialized state so start() can be called explicitly.
 *
 * @return true when firmware initialization and configuration succeeded.
 */
bool VL53L5CX::reset() {
    if (config == nullptr || device == nullptr || state == State::UNINITIALIZED) {
        LogError("%s: Cannot reset VL53L5CX in current state", getName());
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire VL53L5CX mutex while resetting", getName());
        return false;
    }

    const bool was_running = state == State::RUNNING;
    if (was_running) {
        const uint8_t stop_status = vl53l5cx_stop_ranging(device);
        if (stop_status != VL53L5CX_STATUS_OK) {
            logApiError("stop ranging before reset", stop_status);
        }
    }

    setConnectionState(ConnectionState::CONNECTING);
    interrupt_pending = false;
    read_retry_pending = false;
    bool success = setEnabled(false);
    sensor_initialized = false;
    osDelay(VL53L5CX_LPN_RESET_DELAY_MS);
    success = success && initializeSensor();
    if (success) {
        success = configureSensor(was_running);
    }

    if (success) {
        setState(was_running ? State::RUNNING : State::INITIALIZED);
        setConnectionState(ConnectionState::CONNECTED);
        consecutive_read_failures = 0;
        LogInfo("%s: VL53L5CX reset and reconfigured", getName());
    } else {
        (void)setEnabled(false);
        setConnectionState(ConnectionState::DISCONNECTED);
        setState(State::ERROR);
        LogError("%s: Failed to reset VL53L5CX", getName());
    }

    unlock();
    return success;
}

/**
 * @brief Probe the sensor at its active I2C address.
 * @return true when the expected device and revision IDs respond.
 */
bool VL53L5CX::checkConnection() {
    if (device == nullptr || !lock()) {
        return false;
    }

    uint8_t alive = 0;
    const uint8_t status = vl53l5cx_is_alive(device, &alive);
    const bool connected = status == VL53L5CX_STATUS_OK && alive != 0U;
    setConnectionState(connected ? ConnectionState::CONNECTED : ConnectionState::DISCONNECTED);
    if (status != VL53L5CX_STATUS_OK) {
        logApiError("connection check", status);
    }

    unlock();
    return connected;
}

/**
 * @brief Select and, when possible, immediately apply crosstalk calibration.
 * @param calibration Persistent calibration data retained by the driver.
 * @return true when the calibration is valid and was accepted.
 */
bool VL53L5CX::setCalibration(const Calibration& calibration) {
    if (!calibration.valid || calibration.margin_kcps_per_spad > 10000U) {
        LogError("%s: Invalid VL53L5CX crosstalk calibration", getName());
        return false;
    }
    if (state == State::RUNNING) {
        LogError("%s: Stop VL53L5CX before changing crosstalk calibration", getName());
        return false;
    }

    if (!lock()) {
        return false;
    }

    this->calibration = &calibration;
    const bool success = device == nullptr || connection_state != ConnectionState::CONNECTED || applyCalibration();
    unlock();
    return success;
}

/**
 * @brief Run the vendor crosstalk calibration procedure.
 *
 * Ranging must be stopped. The target must fill the complete FoV throughout
 * the potentially multi-second calibration operation.
 *
 * @param settings Target properties and sample count used by the ULD.
 * @param calibration Persistent destination retained after success.
 * @return true when calibration data and margin were read successfully.
 */
bool VL53L5CX::calibrateCrosstalk(const CalibrationSettings& settings, Calibration& calibration) {
    if (state != State::INITIALIZED || connection_state != ConnectionState::CONNECTED) {
        LogError("%s: VL53L5CX must be initialized and stopped before crosstalk calibration", getName());
        return false;
    }
    if (settings.target_distance_mm < 600U || settings.target_distance_mm > 3000U || settings.target_reflectance_percent < 1U ||
        settings.target_reflectance_percent > 99U || settings.sample_count < 1U || settings.sample_count > 16U) {
        LogError("%s: Invalid VL53L5CX crosstalk calibration settings", getName());
        return false;
    }
    if (!lock()) {
        return false;
    }

    uint8_t status = vl53l5cx_calibrate_xtalk(
        device, settings.target_reflectance_percent, settings.sample_count, settings.target_distance_mm);
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_get_caldata_xtalk(device, calibration.crosstalk_data.data());
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_get_xtalk_margin(device, &calibration.margin_kcps_per_spad);
    }
    if (status == VL53L5CX_STATUS_OK) {
        calibration.valid = true;
        this->calibration = &calibration;
        LogInfo("%s: VL53L5CX crosstalk calibration completed", getName());
    } else {
        calibration.valid = false;
        logApiError("crosstalk calibration", status);
    }

    unlock();
    return status == VL53L5CX_STATUS_OK;
}

/**
 * @brief Read the currently active crosstalk calibration from the sensor.
 * @param calibration Destination for the calibration data and margin.
 * @return true when the complete calibration was read.
 */
bool VL53L5CX::getCalibration(Calibration& calibration) {
    if (state != State::INITIALIZED || device == nullptr || connection_state != ConnectionState::CONNECTED || !lock()) {
        LogError("%s: Stop VL53L5CX before reading crosstalk calibration", getName());
        return false;
    }

    uint8_t status = vl53l5cx_get_caldata_xtalk(device, calibration.crosstalk_data.data());
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_get_xtalk_margin(device, &calibration.margin_kcps_per_spad);
    }
    calibration.valid = status == VL53L5CX_STATUS_OK;
    if (!calibration.valid) {
        logApiError("read crosstalk calibration", status);
    }

    unlock();
    return calibration.valid;
}

/**
 * @brief Get the configured scene grid and full detection FoV.
 * @return 4x4 or 8x8 row-major geometry in radians.
 */
TofGeometry VL53L5CX::getGeometry() const {
    const bool is_4x4 = config != nullptr && config->resolution == Resolution::GRID_4X4;
    const uint8_t dimension = is_4x4 ? 4U : 8U;
    return {dimension, dimension, VL53L5CX_HORIZONTAL_FOV_RAD, VL53L5CX_VERTICAL_FOV_RAD};
}

/**
 * @brief Check whether a new frame or failed-read retry is pending.
 *
 * Interrupt mode consumes the edge latched by thread_safe_gpio. Polling mode
 * asks the ULD to compare its stream counter with the previous frame.
 *
 * @return true when readSample() should be called.
 */
bool VL53L5CX::isSampleReady() {
    if (state != State::RUNNING || device == nullptr || !lock()) {
        return false;
    }

    bool ready = read_retry_pending;
    if (!ready && interrupt_initialized) {
        ready = interrupt_pending;
    } else if (!ready) {
        uint8_t data_ready = 0;
        const uint8_t status = vl53l5cx_check_data_ready(device, &data_ready);
        ready = status == VL53L5CX_STATUS_OK && data_ready != 0U;
        if (status != VL53L5CX_STATUS_OK) {
            handleReadResult(false, status);
        }
    }

    unlock();
    return ready;
}

/**
 * @brief Read and normalize one complete ranging frame.
 *
 * Vendor target quality is converted into the generic validity mask, while
 * native status and reflectance are retained for diagnostics. Zone data is
 * reordered into scene-oriented rows before being returned.
 *
 * @param sample Destination for distances, quality, temperature, and timestamp.
 * @return true when the complete frame was transferred and decoded.
 */
bool VL53L5CX::readSample(TofSample& sample) {
    if (state != State::RUNNING || device == nullptr) {
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire VL53L5CX mutex while reading sample", getName());
        return false;
    }

    VL53L5CX_ResultsData results{};
    read_retry_pending = false;
    if (interrupt_initialized) {
        // Clear before reading so an interrupt arriving during the transfer is retained.
        interrupt_pending = false;
    }
    const uint8_t status = vl53l5cx_get_ranging_data(device, &results);
    const bool success = status == VL53L5CX_STATUS_OK;
    if (success) {
        const TofGeometry geometry = getGeometry();
        sample.zone_count = static_cast<uint8_t>(geometry.rows * geometry.columns);
        sample.temperature_c = results.silicon_temp_degc;
        sample.timestamp_ms = osKernelGetTickCount();
        for (size_t row = 0; row < geometry.rows; ++row) {
            for (size_t column = 0; column < geometry.columns; ++column) {
                const size_t destination = row * geometry.columns + column;
                // ST zone 0 observes the top-right of the scene. Mirror each
                // native row into the generic top-left-to-bottom-right order.
                const size_t source = row * geometry.columns + (geometry.columns - 1U - column);
                sample.distance_mm[destination] = results.distance_mm[source];
                sample.status[destination] = results.target_status[source];
                sample.valid[destination] = results.nb_target_detected[source] > 0U &&
                                            (results.target_status[source] == VL53L5CX_TARGET_STATUS_VALID ||
                                             results.target_status[source] == VL53L5CX_TARGET_STATUS_VALID_LARGE_PULSE);
                sample.reflectance_percent[destination] = results.reflectance[source];
            }
        }
    }

    read_retry_pending = !success;
    handleReadResult(success, status);
    unlock();
    return success;
}

/**
 * @brief Get the current consecutive frame-read failure count.
 * @return Number of failures since the most recent successful read.
 */
uint32_t VL53L5CX::getReadFailures() const {
    if (!lock()) {
        return 0;
    }
    const uint32_t failures = consecutive_read_failures;
    unlock();
    return failures;
}

bool VL53L5CX::acquireDeviceContext() {
    for (size_t i = 0; i < VL53L5CX_MAX_INSTANCES; ++i) {
        if (!vl53l5cx_device_slots[i]) {
            vl53l5cx_device_slots[i] = true;
            vl53l5cx_device_buses[i] = config->hi2c;
            vl53l5cx_device_addresses[i] = config->address;
            vl53l5cx_lpn_ports[i] = config->lpn_port;
            vl53l5cx_lpn_pins[i] = config->lpn_pin;
            vl53l5cx_device_enabled[i] = false;
            device_slot = i;
            device = &vl53l5cx_device_pool[i];
            std::memset(device, 0, sizeof(*device));
            return true;
        }
    }
    return false;
}

void VL53L5CX::releaseDeviceContext() {
    if (device_slot < VL53L5CX_MAX_INSTANCES) {
        std::memset(&vl53l5cx_device_pool[device_slot], 0, sizeof(VL53L5CX_Configuration));
        vl53l5cx_device_slots[device_slot] = false;
        vl53l5cx_device_buses[device_slot] = nullptr;
        vl53l5cx_device_addresses[device_slot] = 0U;
        vl53l5cx_lpn_ports[device_slot] = nullptr;
        vl53l5cx_lpn_pins[device_slot] = 0U;
        vl53l5cx_device_enabled[device_slot] = false;
    }
    device_slot = VL53L5CX_MAX_INSTANCES;
    device = nullptr;
}

bool VL53L5CX::initMutex() {
    mutex_attributes.name = getName();
    mutex_attributes.attr_bits = osMutexRecursive;
    mutex_attributes.cb_mem = &mutex_control_block;
    mutex_attributes.cb_size = sizeof(mutex_control_block);
    mutex_id = osMutexNew(&mutex_attributes);
    return mutex_id != nullptr;
}

bool VL53L5CX::initGpios() {
    if (config->lpn_port == nullptr || !isSingleGpioPin(config->lpn_pin)) {
        LogError("%s: VL53L5CX LPn GPIO is required", getName());
        return false;
    }

    GPIO_InitTypeDef lpn_config{};
    lpn_config.Pin = config->lpn_pin;
    lpn_config.Mode = GPIO_MODE_OUTPUT_PP;
    lpn_config.Pull = GPIO_PULLUP;
    lpn_config.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(config->lpn_port, &lpn_config);
    HAL_GPIO_WritePin(config->lpn_port, config->lpn_pin, GPIO_PIN_RESET);
    lpn_initialized = true;

    if (config->interrupt_port != nullptr) {
        if (!GPIO_EXTI_RegisterCallback(config->interrupt_port, config->interrupt_pin, handleInterrupt, this)) {
            LogError("%s: Failed to register VL53L5CX interrupt pin", getName());
            return false;
        }
        interrupt_registered = true;
        interrupt_pending = false;

        GPIO_InitTypeDef interrupt_config{};
        interrupt_config.Pin = config->interrupt_pin;
        interrupt_config.Mode = GPIO_MODE_IT_FALLING;
        interrupt_config.Pull = GPIO_PULLUP;
        interrupt_config.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(config->interrupt_port, &interrupt_config);
        __HAL_GPIO_EXTI_CLEAR_IT(config->interrupt_pin);
        interrupt_pending = false;
        interrupt_initialized = true;
    }
    return true;
}

void VL53L5CX::releaseInterrupt() {
    if (interrupt_registered && config != nullptr) {
        (void)GPIO_EXTI_UnregisterCallback(config->interrupt_port, config->interrupt_pin, handleInterrupt, this);
    }
    interrupt_registered = false;
    interrupt_initialized = false;
    interrupt_pending = false;
    read_retry_pending = false;
}

bool VL53L5CX::initializeSensor() {
    if (!setEnabled(true)) {
        return false;
    }

    // Probe the configured address first to support MCU restarts without a sensor power cycle.
    const bool found_at_configured_address = probeAtAddress(config->address);
    if (!found_at_configured_address && config->address != VL53L5CX_DEFAULT_ADDRESS) {
        for (size_t i = 0; i < VL53L5CX_MAX_INSTANCES; ++i) {
            if (i != device_slot && vl53l5cx_device_slots[i] && vl53l5cx_device_enabled[i] &&
                vl53l5cx_device_buses[i] == config->hi2c && vl53l5cx_device_addresses[i] == VL53L5CX_DEFAULT_ADDRESS) {
                LogError("%s: Start the default-address VL53L5CX last on this I2C bus", getName());
                return false;
            }
        }
    }
    if (!found_at_configured_address && !probeAtAddress(VL53L5CX_DEFAULT_ADDRESS)) {
        LogError("%s: VL53L5CX not found at 0x%02X or default address", getName(), config->address);
        return false;
    }

    uint8_t status = vl53l5cx_init(device);
    if (status != VL53L5CX_STATUS_OK) {
        logApiError("firmware initialization", status);
        return false;
    }

    const uint8_t current_address = static_cast<uint8_t>(device->platform.address >> 1U);
    if (current_address != config->address) {
        status = vl53l5cx_set_i2c_address(device, static_cast<uint16_t>(config->address) << 1U);
        if (status != VL53L5CX_STATUS_OK) {
            logApiError("I2C address assignment", status);
            return false;
        }
    }
    sensor_initialized = applyCalibration();
    return sensor_initialized;
}

bool VL53L5CX::configureSensor(bool start_ranging) {
    uint8_t status = vl53l5cx_set_resolution(device, static_cast<uint8_t>(config->resolution));
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_ranging_frequency_hz(device, config->ranging_frequency_hz);
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_ranging_mode(device, static_cast<uint8_t>(config->ranging_mode));
    }
    if (status == VL53L5CX_STATUS_OK && config->ranging_mode == RangingMode::AUTONOMOUS && config->integration_time_ms != 0U) {
        status = vl53l5cx_set_integration_time_ms(device, config->integration_time_ms);
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_VHV_repeat_count(device, config->vhv_repeat_count);
    }
    if (status == VL53L5CX_STATUS_OK && start_ranging) {
        interrupt_pending = false;
        read_retry_pending = false;
        status = vl53l5cx_start_ranging(device);
    }
    if (status != VL53L5CX_STATUS_OK) {
        logApiError("ranging configuration", status);
        return false;
    }
    return true;
}

bool VL53L5CX::applyCalibration() {
    if (calibration == nullptr || !calibration->valid) {
        return true;
    }

    uint8_t status = vl53l5cx_set_caldata_xtalk(device, const_cast<uint8_t*>(calibration->crosstalk_data.data()));
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_xtalk_margin(device, calibration->margin_kcps_per_spad);
    }
    if (status != VL53L5CX_STATUS_OK) {
        logApiError("apply crosstalk calibration", status);
        return false;
    }
    return true;
}

bool VL53L5CX::setEnabled(bool enabled) {
    if (!lpn_initialized) {
        return false;
    }
    HAL_GPIO_WritePin(config->lpn_port, config->lpn_pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (device_slot < VL53L5CX_MAX_INSTANCES) {
        vl53l5cx_device_enabled[device_slot] = enabled;
    }
    if (!enabled) {
        interrupt_pending = false;
        read_retry_pending = false;
    }
    if (enabled) {
        osDelay(VL53L5CX_LPN_BOOT_DELAY_MS);
    }
    return true;
}

bool VL53L5CX::probeAtAddress(uint8_t address) {
    device->platform.address = static_cast<uint16_t>(address) << 1U;
    uint8_t alive = 0;
    return vl53l5cx_is_alive(device, &alive) == VL53L5CX_STATUS_OK && alive != 0U;
}

bool VL53L5CX::validateConfig(const Config& config) const {
    if (config.hi2c == nullptr || (config.hi2c->Instance != I2C1 && config.hi2c->Instance != I2C2)) {
        LogError("%s: VL53L5CX requires I2C1 or I2C2", getName());
        return false;
    }
    if (config.address < 0x08U || config.address > 0x77U) {
        LogError("%s: Invalid VL53L5CX I2C address 0x%02X", getName(), config.address);
        return false;
    }
    const uint8_t max_frequency = config.resolution == Resolution::GRID_4X4 ? 60U : 15U;
    if (config.ranging_frequency_hz == 0U || config.ranging_frequency_hz > max_frequency) {
        LogError("%s: Invalid VL53L5CX ranging frequency %u Hz", getName(), config.ranging_frequency_hz);
        return false;
    }
    if (config.ranging_mode == RangingMode::AUTONOMOUS && config.integration_time_ms != 0U &&
        (config.integration_time_ms < 2U || config.integration_time_ms > 1000U)) {
        LogError("%s: Invalid VL53L5CX integration time %lu ms", getName(), config.integration_time_ms);
        return false;
    }
    if (config.ranging_mode == RangingMode::CONTINUOUS && config.integration_time_ms != 0U) {
        LogError("%s: VL53L5CX integration time only applies in autonomous mode", getName());
        return false;
    }
    if (config.integration_time_ms != 0U) {
        const uint32_t integrations_per_frame = config.resolution == Resolution::GRID_4X4 ? 1U : 4U;
        const uint64_t frame_time_ms = static_cast<uint64_t>(config.integration_time_ms) * integrations_per_frame + 1U;
        if (frame_time_ms * config.ranging_frequency_hz >= 1000U) {
            LogError("%s: VL53L5CX integration window is too long for the ranging frequency", getName());
            return false;
        }
    }
    if (config.read_failure_threshold == 0U) {
        LogError("%s: VL53L5CX read failure threshold must be nonzero", getName());
        return false;
    }
    if (config.lpn_port == nullptr || !isSingleGpioPin(config.lpn_pin)) {
        LogError("%s: VL53L5CX requires one valid LPn GPIO pin", getName());
        return false;
    }
    const bool interrupt_port_configured = config.interrupt_port != nullptr;
    const bool interrupt_pin_configured = config.interrupt_pin != 0U;
    if (interrupt_port_configured != interrupt_pin_configured ||
        (interrupt_pin_configured && !isSingleGpioPin(config.interrupt_pin))) {
        LogError("%s: Invalid VL53L5CX interrupt GPIO configuration", getName());
        return false;
    }
    if (interrupt_port_configured && config.interrupt_port == config.lpn_port && config.interrupt_pin == config.lpn_pin) {
        LogError("%s: VL53L5CX LPn and interrupt GPIOs must be different", getName());
        return false;
    }
    return true;
}

bool VL53L5CX::lock() const { return mutex_id != nullptr && osMutexAcquire(mutex_id, osWaitForever) == osOK; }

void VL53L5CX::unlock() const {
    if (osMutexRelease(mutex_id) != osOK) {
        LogError("%s: Failed to release VL53L5CX mutex", getName());
    }
}

void VL53L5CX::handleReadResult(bool success, uint8_t status) {
    if (success) {
        if (consecutive_read_failures >= config->read_failure_threshold) {
            LogInfo("%s: VL53L5CX readings recovered", getName());
        }
        consecutive_read_failures = 0;
        last_read_status = VL53L5CX_STATUS_OK;
        setConnectionState(ConnectionState::CONNECTED);
        return;
    }

    if (consecutive_read_failures < UINT32_MAX) {
        consecutive_read_failures++;
    }
    last_read_status = status;
    if (consecutive_read_failures == config->read_failure_threshold) {
        LogError("%s: VL53L5CX read failure threshold reached (%lu), last status %u", getName(), consecutive_read_failures, last_read_status);
        setConnectionState(ConnectionState::DISCONNECTED);
    }
}

void VL53L5CX::logApiError(const char* operation, uint8_t status) const {
    LogError("%s: VL53L5CX %s failed with status %u", getName(), operation, status);
}

void VL53L5CX::handleInterrupt(GPIO_TypeDef* port, uint16_t pin, void* context) {
    auto* driver = static_cast<VL53L5CX*>(context);
    if (driver != nullptr && driver->config != nullptr && port == driver->config->interrupt_port &&
        pin == driver->config->interrupt_pin) {
        driver->interrupt_pending = true;
    }
}
