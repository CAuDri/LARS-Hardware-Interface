/**
 * @file bno055.cpp
 *
 * @brief CAuDri - Bosch BNO055 IMU driver implementation
 */

#include "bno055.hpp"

#include <array>

#include "logger.h"
#include "thread_safe_i2c.h"

static constexpr uint32_t BNO055_CHIP_ID_POLL_MS = 10;
static constexpr uint32_t BNO055_SOFT_RESET_SETTLE_MS = 50;
static constexpr uint32_t BNO055_HARD_RESET_LOW_MS = 1;
static constexpr uint32_t BNO055_HARD_RESET_BOOT_MS = 800;
static constexpr uint32_t BNO055_MODE_SWITCH_DELAY_MS = 30;
static constexpr uint32_t BNO055_POWER_MODE_DELAY_MS = 10;
static constexpr uint32_t BNO055_CONFIG_WRITE_DELAY_MS = 10;
static constexpr uint32_t BNO055_EXTERNAL_CRYSTAL_STARTUP_MS = 650;

static constexpr float BNO055_ACCEL_SCALE = 100.0F;
static constexpr float BNO055_MAG_SCALE = 16.0F;
static constexpr float BNO055_GYRO_RAD_SCALE = 900.0F;
static constexpr float BNO055_QUAT_SCALE = 16384.0F;

static bool isSingleGpioPin(uint16_t pin) { return pin != 0U && pin <= GPIO_PIN_15 && (pin & (pin - 1U)) == 0U; }

/**
 * @brief Construct an uninitialized BNO055 driver.
 *
 * @param name Driver name used for diagnostics and logging.
 */
BNO055::BNO055(const char* name) : ImuDriver(name) {}

/**
 * @brief Construct and initialize a BNO055 driver.
 *
 * @param name Driver name used for diagnostics and logging.
 * @param config Hardware and sensor configuration.
 */
BNO055::BNO055(const char* name, const Config& config) : ImuDriver(name) { init(config); }

/**
 * @brief Destroy the BNO055 driver.
 */
BNO055::~BNO055() = default;

/**
 * @brief Initialize the sensor, GPIOs, mutex, and BNO055 configuration.
 *
 * @param config Hardware and sensor configuration.
 * @return true when the device was reset and configured successfully.
 */
bool BNO055::init(const Config& config) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Cannot initialize, driver not uninitialized", getName());
        return false;
    }
    if (config.hi2c == nullptr) {
        LogError("%s: BNO055 I2C handle is not configured", getName());
        setState(State::ERROR);
        return false;
    }
    if (config.hi2c->Instance != I2C1 && config.hi2c->Instance != I2C2) {
        LogError("%s: BNO055 requires I2C1 or I2C2", getName());
        setState(State::ERROR);
        return false;
    }
    if (config.address != BNO055_I2C_ADDR_LO && config.address != BNO055_I2C_ADDR_HI) {
        LogError("%s: Invalid BNO055 I2C address 0x%02X", getName(), config.address);
        setState(State::ERROR);
        return false;
    }
    if (config.boot_timeout_ms == 0U || config.read_failure_threshold == 0U) {
        LogError("%s: BNO055 timeouts and failure threshold must be nonzero", getName());
        setState(State::ERROR);
        return false;
    }
    if (!isAxisMapValid(config.axis_map)) {
        LogError("%s: Invalid BNO055 axis map (config 0x%02X, sign 0x%02X)",
                 getName(),
                 config.axis_map.config,
                 config.axis_map.sign);
        setState(State::ERROR);
        return false;
    }

    this->config = config;

    if (!initMutex()) {
        LogError("%s: Failed to create BNO055 mutex", getName());
        setState(State::ERROR);
        return false;
    }

    if (config.reset_port != nullptr) {
        reset_pin_initialized = initResetPin();
        if (!reset_pin_initialized) {
            LogWarning("%s: Invalid reset pin configuration, using software reset", getName());
        }
    } else {
        LogDebug("%s: No reset pin configured, using software reset", getName());
    }
    if (config.interrupt_port != nullptr || config.interrupt_pin != 0U) {
        interrupt_pin_initialized = initInterruptPin();
        if (!interrupt_pin_initialized) {
            LogWarning("%s: Invalid interrupt pin configuration, using polling only", getName());
        }
    } else {
        LogDebug("%s: No interrupt pin configured, polling only", getName());
    }

    if (!reset()) {
        LogError("%s: Failed to initialize BNO055", getName());
        return false;
    }

    LogInfo("%s: BNO055 initialized at 0x%02X", getName(), config.address);
    return true;
}

/**
 * @brief Verify the sensor and enter the configured operation mode.
 *
 * @return true when the driver entered the running state.
 */
bool BNO055::start() {
    if (state != State::INITIALIZED) {
        LogError("%s: Cannot start, driver not initialized", getName());
        return false;
    }

    setConnectionState(ConnectionState::CONNECTING);
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while starting", getName());
        setConnectionState(ConnectionState::DISCONNECTED);
        return false;
    }

    const bool success = checkConnection() && setOperationMode(config.operation_mode);
    unlock();
    if (!success) {
        LogError("%s: Failed to start BNO055", getName());
        setConnectionState(ConnectionState::DISCONNECTED);
        return false;
    }

    setState(State::RUNNING);
    setConnectionState(ConnectionState::CONNECTED);
    LogInfo("%s: BNO055 started", getName());
    return true;
}

/**
 * @brief Reset and reconfigure the BNO055.
 *
 * A running driver returns to its configured operation mode. Otherwise, a
 * successful reset leaves the driver initialized and ready to start.
 *
 * @return true when reset and reconfiguration completed successfully.
 */
bool BNO055::reset() {
    if (config.hi2c == nullptr) {
        LogError("%s: Cannot reset BNO055 without an I2C handle", getName());
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while resetting", getName());
        return false;
    }

    const State previous_state = state;
    const bool restore_running_mode = previous_state == State::RUNNING;
    setConnectionState(ConnectionState::CONNECTING);

    const bool success = resetDevice() && configureDevice() && (!restore_running_mode || setOperationMode(config.operation_mode));
    if (success) {
        consecutive_read_failures = 0;
        setState(restore_running_mode ? State::RUNNING : State::INITIALIZED);
        setConnectionState(ConnectionState::CONNECTED);
        if (previous_state != State::UNINITIALIZED) {
            LogInfo("%s: BNO055 reset and reconfigured", getName());
        }
    } else {
        LogError("%s: Failed to reset and reconfigure BNO055", getName());
        setState(State::ERROR);
        setConnectionState(ConnectionState::DISCONNECTED);
    }

    unlock();
    return success;
}

/**
 * @brief Check whether the expected BNO055 chip ID can be read.
 *
 * @return true when the sensor responds with the expected chip ID.
 */
bool BNO055::checkConnection() {
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while checking connection", getName());
        setConnectionState(ConnectionState::DISCONNECTED);
        return false;
    }

    uint8_t chip_id = 0;
    const bool read_success = setPage(0) && readRegister(CHIP_ID, chip_id);
    const bool connected = read_success && chip_id == BNO055_CHIP_ID_VALUE;
    if (!read_success) {
        LogError("%s: Failed to read BNO055 chip ID", getName());
    } else if (!connected) {
        LogError("%s: Unexpected BNO055 chip ID 0x%02X", getName(), chip_id);
    }
    setConnectionState(connected ? ConnectionState::CONNECTED : ConnectionState::DISCONNECTED);
    unlock();
    return connected;
}

/**
 * @brief Check whether a sample is ready according to the interrupt pin.
 *
 * @return true when polling is configured or the data-ready pin is asserted.
 */
bool BNO055::isSampleReady() const {
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while checking sample readiness", getName());
        return false;
    }

    const bool ready = !data_ready_interrupt_configured || HAL_GPIO_ReadPin(config.interrupt_port, config.interrupt_pin) == GPIO_PIN_SET;
    unlock();
    return ready;
}

/**
 * @brief Read and scale one acceleration, angular velocity, magnetic field, and orientation sample.
 *
 * @param sample Destination for the converted sample.
 * @return true when the complete sensor register block was read successfully.
 */
bool BNO055::readSample(ImuSample& sample) {
    if (state != State::RUNNING) {
        LogError("%s: Cannot read BNO055 sample while driver is not running", getName());
        return false;
    }
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading sample", getName());
        return false;
    }

    bool success = setPage(0);
    // One burst covers the contiguous acceleration, magnetic field, gyro, and quaternion registers.
    std::array<uint8_t, SENSOR_DATA_LENGTH> data{};
    if (success) {
        success = readRegisters(ACC_DATA_X_LSB, data.data(), data.size());
    }

    if (success) {
        sample.acceleration_mps2[0] = static_cast<float>(readInt16(data.data(), 0)) / BNO055_ACCEL_SCALE;
        sample.acceleration_mps2[1] = static_cast<float>(readInt16(data.data(), 2)) / BNO055_ACCEL_SCALE;
        sample.acceleration_mps2[2] = static_cast<float>(readInt16(data.data(), 4)) / BNO055_ACCEL_SCALE;

        sample.magnetic_field_ut[0] = static_cast<float>(readInt16(data.data(), 6)) / BNO055_MAG_SCALE;
        sample.magnetic_field_ut[1] = static_cast<float>(readInt16(data.data(), 8)) / BNO055_MAG_SCALE;
        sample.magnetic_field_ut[2] = static_cast<float>(readInt16(data.data(), 10)) / BNO055_MAG_SCALE;

        sample.angular_velocity_radps[0] = static_cast<float>(readInt16(data.data(), 12)) / BNO055_GYRO_RAD_SCALE;
        sample.angular_velocity_radps[1] = static_cast<float>(readInt16(data.data(), 14)) / BNO055_GYRO_RAD_SCALE;
        sample.angular_velocity_radps[2] = static_cast<float>(readInt16(data.data(), 16)) / BNO055_GYRO_RAD_SCALE;

        // The sensor stores W first; ImuSample follows the ROS x, y, z, w convention.
        sample.orientation_xyzw[3] = static_cast<float>(readInt16(data.data(), 24)) / BNO055_QUAT_SCALE;
        sample.orientation_xyzw[0] = static_cast<float>(readInt16(data.data(), 26)) / BNO055_QUAT_SCALE;
        sample.orientation_xyzw[1] = static_cast<float>(readInt16(data.data(), 28)) / BNO055_QUAT_SCALE;
        sample.orientation_xyzw[2] = static_cast<float>(readInt16(data.data(), 30)) / BNO055_QUAT_SCALE;
        sample.timestamp_ms = osKernelGetTickCount();

        if (data_ready_interrupt_configured && !clearInterrupt()) {
            LogError("%s: Failed to clear BNO055 data-ready interrupt", getName());
        }
    }

    handleReadResult(success);
    unlock();
    return success;
}

/**
 * @brief Return the active BNO055 configuration.
 *
 * @return Immutable driver configuration.
 */
const BNO055::Config& BNO055::getConfig() const { return config; }

/**
 * @brief Read the current BNO055 operation mode.
 *
 * @return Reported operation mode, or the last known mode when the read fails.
 */
BNO055::OperationMode BNO055::getOperationMode() {
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading operation mode", getName());
        return active_mode;
    }

    uint8_t mode = 0;
    if (!setPage(0) || !readRegister(OPR_MODE, mode)) {
        LogError("%s: Failed to read BNO055 operation mode", getName());
        unlock();
        return active_mode;
    }
    active_mode = static_cast<OperationMode>(mode & 0x0F);
    unlock();
    return active_mode;
}

/**
 * @brief Read the four BNO055 calibration levels.
 *
 * @return Calibration levels from zero to three, or all zeros on failure.
 */
BNO055::CalibrationStatus BNO055::getCalibrationStatus() {
    CalibrationStatus status{};
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading calibration", getName());
        return status;
    }

    uint8_t raw = 0;
    if (!setPage(0) || !readRegister(CALIB_STAT, raw)) {
        LogError("%s: Failed to read BNO055 calibration status", getName());
        unlock();
        return status;
    }

    status.system = (raw >> 6U) & 0x03U;
    status.gyro = (raw >> 4U) & 0x03U;
    status.accel = (raw >> 2U) & 0x03U;
    status.mag = raw & 0x03U;
    unlock();
    return status;
}

/**
 * @brief Read the BNO055 system, self-test, and error status registers.
 *
 * @return Raw status register values; failed fields remain zero.
 */
BNO055::Status BNO055::getStatus() {
    Status status{};
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading status", getName());
        return status;
    }

    const bool success = setPage(0) && readRegister(SYS_STATUS, status.system) &&
                         readRegister(ST_RESULT, status.self_test) && readRegister(SYS_ERR, status.error);
    unlock();
    if (!success) {
        LogError("%s: Failed to read BNO055 status registers", getName());
    }
    return status;
}

/**
 * @brief Read the BNO055 temperature in degrees Celsius.
 *
 * @return Sensor temperature, or zero when the read fails.
 */
int8_t BNO055::getTemperature() {
    uint8_t temperature = 0;
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading temperature", getName());
        return 0;
    }

    const bool success = setPage(0) && readRegister(TEMP, temperature);
    unlock();
    if (!success) {
        LogError("%s: Failed to read BNO055 temperature", getName());
    }
    return static_cast<int8_t>(temperature);
}

/**
 * @brief Return the number of consecutive failed sample reads.
 *
 * @return Consecutive read failure count.
 */
uint32_t BNO055::getReadFailures() const {
    if (!lock()) {
        LogError("%s: Failed to acquire BNO055 mutex while reading failure count", getName());
        return 0;
    }

    const uint32_t failures = consecutive_read_failures;
    unlock();
    return failures;
}

/**
 * @brief Read the configured data-ready GPIO level.
 *
 * @return true when the initialized interrupt pin is high.
 */
bool BNO055::isInterruptTriggered() const {
    if (!interrupt_pin_initialized) {
        return false;
    }
    return HAL_GPIO_ReadPin(config.interrupt_port, config.interrupt_pin) == GPIO_PIN_SET;
}

bool BNO055::initMutex() {
    mutex_attributes.name = getName();
    mutex_attributes.attr_bits = osMutexRecursive;
    mutex_attributes.cb_mem = &mutex_control_block;
    mutex_attributes.cb_size = sizeof(mutex_control_block);
    mutex_id = osMutexNew(&mutex_attributes);
    return mutex_id != nullptr;
}

bool BNO055::initResetPin() {
    if (config.reset_port == nullptr || config.reset_pin == 0U) {
        return false;
    }

    GPIO_InitTypeDef reset_config{};
    reset_config.Pin = config.reset_pin;
    reset_config.Mode = GPIO_MODE_OUTPUT_PP;
    reset_config.Pull = GPIO_PULLUP;
    reset_config.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(config.reset_port, &reset_config);
    HAL_GPIO_WritePin(config.reset_port, config.reset_pin, GPIO_PIN_SET);
    return true;
}

bool BNO055::initInterruptPin() {
    if (config.interrupt_port == nullptr || !isSingleGpioPin(config.interrupt_pin)) {
        return false;
    }

    GPIO_InitTypeDef interrupt_config{};
    interrupt_config.Pin = config.interrupt_pin;
    interrupt_config.Mode = GPIO_MODE_INPUT;
    interrupt_config.Pull = GPIO_NOPULL;
    interrupt_config.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(config.interrupt_port, &interrupt_config);
    return true;
}

bool BNO055::resetDevice() {
    data_ready_interrupt_configured = false;

    if (reset_pin_initialized) {
        HAL_GPIO_WritePin(config.reset_port, config.reset_pin, GPIO_PIN_RESET);
        osDelay(BNO055_HARD_RESET_LOW_MS);
        HAL_GPIO_WritePin(config.reset_port, config.reset_pin, GPIO_PIN_SET);
        osDelay(BNO055_HARD_RESET_BOOT_MS);
    } else {
        if (!setPage(0) || !writeRegister(SYS_TRIGGER, SYS_TRIGGER_RESET)) {
            LogError("%s: Failed to request BNO055 software reset", getName());
            return false;
        }
        osDelay(BNO055_SOFT_RESET_SETTLE_MS);
    }

    if (!waitForChipId(config.boot_timeout_ms)) {
        LogError("%s: BNO055 chip id did not appear after reset", getName());
        return false;
    }

    active_mode = OperationMode::CONFIG;
    return true;
}

bool BNO055::waitForChipId(uint32_t timeout_ms) {
    const uint32_t start_ms = osKernelGetTickCount();
    do {
        uint8_t chip_id = 0;
        if (readRegister(CHIP_ID, chip_id) && chip_id == BNO055_CHIP_ID_VALUE) {
            return true;
        }
        osDelay(BNO055_CHIP_ID_POLL_MS);
    } while (osKernelGetTickCount() - start_ms < timeout_ms);

    return false;
}

bool BNO055::readSoftwareRevision(uint16_t& revision) {
    uint8_t revision_bytes[2]{};
    if (!setPage(0) || !readRegisters(SW_REV_ID_LSB, revision_bytes, sizeof(revision_bytes))) {
        return false;
    }

    revision = static_cast<uint16_t>(revision_bytes[0]) | (static_cast<uint16_t>(revision_bytes[1]) << 8U);
    return true;
}

bool BNO055::configureDevice() {
    if (!setOperationMode(OperationMode::CONFIG)) {
        LogError("%s: Failed to enter BNO055 configuration mode", getName());
        return false;
    }
    if (!writeRegister(PWR_MODE, POWER_MODE_NORMAL)) {
        LogError("%s: Failed to configure BNO055 power mode", getName());
        return false;
    }
    osDelay(BNO055_POWER_MODE_DELAY_MS);

    if (!setPage(0)) {
        LogError("%s: Failed to select BNO055 register page 0", getName());
        return false;
    }
    if (!writeRegister(UNIT_SEL, UNIT_SEL_ANDROID_RADPS_MPS2_C)) {
        LogError("%s: Failed to configure BNO055 units", getName());
        return false;
    }

    if (config.axis_map.configured) {
        if (!writeRegister(AXIS_MAP_CONFIG, config.axis_map.config) || !writeRegister(AXIS_MAP_SIGN, config.axis_map.sign)) {
            LogError("%s: Failed to configure BNO055 axis mapping", getName());
            return false;
        }
        osDelay(BNO055_CONFIG_WRITE_DELAY_MS);
    }

    if (!setExternalCrystal(config.use_external_crystal)) {
        LogError("%s: Failed to configure BNO055 clock source", getName());
        return false;
    }

    if (interrupt_pin_initialized) {
        uint16_t software_revision = 0U;
        if (!readSoftwareRevision(software_revision)) {
            LogWarning("%s: Failed to read BNO055 software revision, using polling only", getName());
        } else if (software_revision < DATA_READY_MIN_SW_REVISION) {
            LogInfo("%s: BNO055 software revision %02X.%02X does not support data-ready interrupts, using polling",
                    getName(),
                    static_cast<unsigned>((software_revision >> 8U) & 0xFFU),
                    static_cast<unsigned>(software_revision & 0xFFU));
        } else {
            data_ready_interrupt_configured = configureDataReadyInterrupt();
            if (!data_ready_interrupt_configured) {
                LogWarning("%s: Failed to configure BNO055 data-ready interrupt, using polling only", getName());
            }
        }
    }

    return setOperationMode(OperationMode::CONFIG);
}

bool BNO055::configureDataReadyInterrupt() {
    const OperationMode previous_mode = active_mode;
    if (!setOperationMode(OperationMode::CONFIG) || !setPage(1)) {
        return false;
    }

    uint8_t int_enable = 0;
    uint8_t int_mask = 0;
    bool success = readRegister(INT_EN, int_enable) && readRegister(INT_MSK, int_mask);
    if (success) {
        int_enable |= INTERRUPT_ACC_BSX_DRDY;
        int_mask |= INTERRUPT_ACC_BSX_DRDY;
        success = writeRegister(INT_EN, int_enable) && writeRegister(INT_MSK, int_mask);
    }

    const bool page_restored = setPage(0);
    const bool interrupt_cleared = page_restored && clearInterrupt();
    const bool mode_restored = setOperationMode(previous_mode);
    return success && page_restored && interrupt_cleared && mode_restored;
}

bool BNO055::clearInterrupt() {
    if (!setPage(0)) {
        return false;
    }

    // SYS_TRIGGER also owns the clock-select bit, so preserve it while acknowledging the interrupt.
    const uint8_t trigger = SYS_TRIGGER_RESET_INTERRUPT | (config.use_external_crystal ? SYS_TRIGGER_EXTERNAL_CRYSTAL : 0U);
    return writeRegister(SYS_TRIGGER, trigger);
}

bool BNO055::setPage(uint8_t page) { return writeRegister(PAGE_ID, page); }

bool BNO055::setOperationMode(OperationMode mode) {
    if (!setPage(0) || !writeRegister(OPR_MODE, static_cast<uint8_t>(mode))) {
        return false;
    }

    osDelay(BNO055_MODE_SWITCH_DELAY_MS);

    uint8_t readback = 0;
    if (!readRegister(OPR_MODE, readback)) {
        return false;
    }

    const bool success = (readback & 0x0FU) == static_cast<uint8_t>(mode);
    if (success) {
        active_mode = mode;
    } else {
        LogError("%s: BNO055 operation mode readback mismatch (requested 0x%02X, read 0x%02X)",
                 getName(),
                 static_cast<uint8_t>(mode),
                 readback & 0x0FU);
    }
    return success;
}

bool BNO055::setExternalCrystal(bool enabled) {
    const OperationMode previous_mode = active_mode;
    if (!setOperationMode(OperationMode::CONFIG) || !setPage(0)) {
        return false;
    }

    const uint8_t trigger = enabled ? SYS_TRIGGER_EXTERNAL_CRYSTAL : 0x00U;
    if (!writeRegister(SYS_TRIGGER, trigger)) {
        return false;
    }

    osDelay(enabled ? BNO055_EXTERNAL_CRYSTAL_STARTUP_MS : BNO055_CONFIG_WRITE_DELAY_MS);

    if (enabled) {
        uint8_t clock_status = 0U;
        uint8_t trigger_readback = 0U;
        if (!readRegister(SYS_CLK_STATUS, clock_status) || !readRegister(SYS_TRIGGER, trigger_readback)) {
            LogError("%s: Failed to verify BNO055 external clock", getName());
            return false;
        }
        if ((clock_status & SYS_CLK_STATUS_MAIN_CLOCK) != 0U || (trigger_readback & SYS_TRIGGER_EXTERNAL_CRYSTAL) == 0U) {
            LogError("%s: BNO055 external crystal failed to start", getName());
            return false;
        }
    }

    return setOperationMode(previous_mode);
}

bool BNO055::writeRegister(Register reg, uint8_t value) { return writeRegisters(reg, &value, 1); }

bool BNO055::readRegister(Register reg, uint8_t& value) { return readRegisters(reg, &value, 1); }

bool BNO055::readRegisters(Register start_reg, uint8_t* data, uint16_t size) {
    return I2C_MemRead8(config.hi2c, i2cAddress(), static_cast<uint8_t>(start_reg), data, size) == HAL_OK;
}

bool BNO055::writeRegisters(Register start_reg, uint8_t* data, uint16_t size) {
    if (data == nullptr || size == 0U) {
        return false;
    }
    return I2C_MemWrite8(config.hi2c, i2cAddress(), static_cast<uint8_t>(start_reg), data, size) == HAL_OK;
}

uint16_t BNO055::i2cAddress() const { return static_cast<uint16_t>(config.address) << 1U; }

bool BNO055::lock() const { return mutex_id != nullptr && osMutexAcquire(mutex_id, osWaitForever) == osOK; }

void BNO055::unlock() const {
    if (osMutexRelease(mutex_id) != osOK) {
        LogError("%s: Failed to release BNO055 mutex", getName());
    }
}

void BNO055::handleReadResult(bool success) {
    if (success) {
        if (consecutive_read_failures >= config.read_failure_threshold) {
            LogInfo("%s: BNO055 readings recovered", getName());
        }
        consecutive_read_failures = 0;
        setConnectionState(ConnectionState::CONNECTED);
        return;
    }

    if (consecutive_read_failures < UINT32_MAX) {
        consecutive_read_failures++;
    }

    if (consecutive_read_failures == config.read_failure_threshold) {
        LogError("%s: BNO055 read failure threshold reached (%lu consecutive failures)", getName(), consecutive_read_failures);
        setConnectionState(ConnectionState::DISCONNECTED);
    }
}

bool BNO055::isAxisMapValid(const AxisMap& axis_map) {
    if (!axis_map.configured) {
        return true;
    }

    const uint8_t x_axis = axis_map.config & 0x03U;
    const uint8_t y_axis = (axis_map.config >> 2U) & 0x03U;
    const uint8_t z_axis = (axis_map.config >> 4U) & 0x03U;
    return (axis_map.config & 0xC0U) == 0U && (axis_map.sign & 0xF8U) == 0U && x_axis < 3U && y_axis < 3U &&
           z_axis < 3U && x_axis != y_axis && x_axis != z_axis && y_axis != z_axis;
}

int16_t BNO055::readInt16(const uint8_t* data, size_t offset) {
    return static_cast<int16_t>(static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1U]) << 8U));
}
