/**
 * @file bno055.hpp
 *
 * @brief CAuDri - Bosch BNO055 IMU driver
 */
#pragma once

#include <cmsis_os2.h>

#include <cstdint>

#include "imu.hpp"
#include "stm32f4xx_hal.h"

constexpr uint8_t BNO055_I2C_ADDR_LO = 0x28;
constexpr uint8_t BNO055_I2C_ADDR_HI = 0x29;

constexpr uint8_t BNO055_CHIP_ID_VALUE = 0xA0;

constexpr uint8_t BNO055_AXIS_MAP_CONFIG_DEFAULT = 0x24;
constexpr uint8_t BNO055_AXIS_MAP_SIGN_DEFAULT = 0x00;

constexpr uint32_t BNO055_DEFAULT_BOOT_TIMEOUT_MS = 1000;
constexpr uint32_t BNO055_DEFAULT_READ_FAILURE_THRESHOLD = 5;

class BNO055 : public ImuDriver {
   public:
    enum class OperationMode : uint8_t {
        CONFIG = 0x00,
        ACCONLY = 0x01,
        MAGONLY = 0x02,
        GYRONLY = 0x03,
        ACCMAG = 0x04,
        ACCGYRO = 0x05,
        MAGGYRO = 0x06,
        AMG = 0x07,
        IMU = 0x08,
        COMPASS = 0x09,
        M4G = 0x0A,
        NDOF_FMC_OFF = 0x0B,
        NDOF = 0x0C,
    };

    struct AxisMap {
        uint8_t config = BNO055_AXIS_MAP_CONFIG_DEFAULT;
        uint8_t sign = BNO055_AXIS_MAP_SIGN_DEFAULT;
        bool configured = false;
    };

    /**
     * @brief Configuration structure for the BNO055 driver
     *
     * @param hi2c Pointer to the I2C handle for communication with the BNO055
     * @param address I2C address of the BNO055 (default: BNO055_I2C_ADDR_LO)
     * @param reset_port GPIO port for the reset pin (optional)
     * @param reset_pin GPIO pin number for the reset pin (optional)
     * @param interrupt_port GPIO port for the interrupt pin (optional)
     * @param interrupt_pin GPIO pin number for the interrupt pin (optional)
     * @param operation_mode Initial operation mode for the BNO055 (default: NDOF)
     * @param use_external_crystal Use external crystal for better accuracy (default: true)
     * @param axis_map Axis mapping configuration (optional)
     * @param boot_timeout_ms Timeout for waiting for the chip ID after reset (default: 1s)
     * @param read_failure_threshold Number of consecutive read failures before considering the device disconnected (default: 5)
     */
    struct Config {
        I2C_HandleTypeDef* hi2c = nullptr;
        uint8_t address = BNO055_I2C_ADDR_LO;

        GPIO_TypeDef* reset_port = nullptr;
        uint16_t reset_pin = 0;

        GPIO_TypeDef* interrupt_port = nullptr;
        uint16_t interrupt_pin = 0;

        OperationMode operation_mode = OperationMode::NDOF;
        bool use_external_crystal = true;
        AxisMap axis_map{};
        uint32_t boot_timeout_ms = BNO055_DEFAULT_BOOT_TIMEOUT_MS;
        uint32_t read_failure_threshold = BNO055_DEFAULT_READ_FAILURE_THRESHOLD;
    };

    struct CalibrationStatus {
        uint8_t system = 0;
        uint8_t gyro = 0;
        uint8_t accel = 0;
        uint8_t mag = 0;
    };

    struct Status {
        uint8_t system = 0;
        uint8_t self_test = 0;
        uint8_t error = 0;
    };

    explicit BNO055(const char* name);
    BNO055(const char* name, const Config& config);
    ~BNO055() override;

    bool init(const Config& config);
    bool start();
    bool reset();
    bool checkConnection();
    bool isSampleReady() const override;
    bool readSample(ImuSample& sample) override;

    const Config& getConfig() const;
    OperationMode getOperationMode();

    CalibrationStatus getCalibrationStatus();
    Status getStatus();
    int8_t getTemperature();

    uint32_t getReadFailures() const;
    bool isInterruptTriggered() const;

   private:
    enum Register : uint8_t {
        CHIP_ID = 0x00,
        SW_REV_ID_LSB = 0x04,
        PAGE_ID = 0x07,
        ACC_DATA_X_LSB = 0x08,
        CALIB_STAT = 0x35,
        ST_RESULT = 0x36,
        SYS_CLK_STATUS = 0x38,
        SYS_STATUS = 0x39,
        SYS_ERR = 0x3A,
        UNIT_SEL = 0x3B,
        OPR_MODE = 0x3D,
        PWR_MODE = 0x3E,
        SYS_TRIGGER = 0x3F,
        TEMP = 0x34,
        AXIS_MAP_CONFIG = 0x41,
        AXIS_MAP_SIGN = 0x42,
        INT_MSK = 0x0F,
        INT_EN = 0x10,
    };

    static constexpr size_t SENSOR_DATA_LENGTH = 32;
    static constexpr uint8_t UNIT_SEL_ANDROID_RADPS_MPS2_C = 0x02;
    static constexpr uint8_t POWER_MODE_NORMAL = 0x00;
    static constexpr uint8_t SYS_TRIGGER_RESET = 0x20;
    static constexpr uint8_t SYS_TRIGGER_RESET_INTERRUPT = 0x40;
    static constexpr uint8_t SYS_TRIGGER_EXTERNAL_CRYSTAL = 0x80;
    static constexpr uint8_t SYS_CLK_STATUS_MAIN_CLOCK = 0x01;
    static constexpr uint8_t INTERRUPT_ACC_BSX_DRDY = 0x01;
    static constexpr uint16_t DATA_READY_MIN_SW_REVISION = 0x0314;

    Config config{};
    bool reset_pin_initialized = false;
    bool interrupt_pin_initialized = false;
    bool data_ready_interrupt_configured = false;
    OperationMode active_mode = OperationMode::CONFIG;
    uint32_t consecutive_read_failures = 0;

    mutable osMutexId_t mutex_id = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};

    bool initMutex();
    bool initResetPin();
    bool initInterruptPin();

    bool resetDevice();

    bool waitForChipId(uint32_t timeout_ms);
    bool readSoftwareRevision(uint16_t& revision);

    bool configureDevice();
    bool configureDataReadyInterrupt();
    bool clearInterrupt();

    bool setPage(uint8_t page);
    bool setOperationMode(OperationMode mode);
    bool setExternalCrystal(bool enabled);

    bool readRegister(Register reg, uint8_t& value);
    bool readRegisters(Register start_reg, uint8_t* data, uint16_t size);
    bool writeRegister(Register reg, uint8_t value);
    bool writeRegisters(Register start_reg, uint8_t* data, uint16_t size);

    uint16_t i2cAddress() const;
    
    bool lock() const;
    void unlock() const;

    void handleReadResult(bool success);
    static bool isAxisMapValid(const AxisMap& axis_map);
    static int16_t readInt16(const uint8_t* data, size_t offset);
};
