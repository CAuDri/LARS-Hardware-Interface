/**
 * @file vl53l5cx.hpp
 *
 * @brief ST VL53L5CX multizone Time-of-Flight driver
 * Based on the vendor provided VL53L5CX API.
 */
#pragma once

#include <cmsis_os2.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "stm32f4xx_hal.h"
#include "tof.hpp"

extern "C" {
#include "vl53l5cx_api.h"
}

constexpr size_t VL53L5CX_MAX_INSTANCES = TOF_MAX_SENSORS;
constexpr uint8_t VL53L5CX_DEFAULT_ADDRESS = 0x29;
constexpr uint32_t VL53L5CX_DEFAULT_FAILURE_THRESHOLD = 5;

// The datasheet specifies a 45 degree horizontal and vertical detection volume.
constexpr float VL53L5CX_HORIZONTAL_FOV_RAD = 45.0F * (3.141592654F / 180.0F);
constexpr float VL53L5CX_VERTICAL_FOV_RAD = 45.0F * (3.141592654F / 180.0F);

class VL53L5CX : public TofDriver {
   public:
    enum class Resolution : uint8_t {
        GRID_4X4 = VL53L5CX_RESOLUTION_4X4,
        GRID_8X8 = VL53L5CX_RESOLUTION_8X8,
    };

    enum class RangingMode : uint8_t {
        CONTINUOUS = VL53L5CX_RANGING_MODE_CONTINUOUS,
        AUTONOMOUS = VL53L5CX_RANGING_MODE_AUTONOMOUS,
    };

    /**
     * @brief
     */
    struct Config {
        I2C_HandleTypeDef* hi2c = nullptr;
        uint8_t address = VL53L5CX_DEFAULT_ADDRESS;

        GPIO_TypeDef* lpn_port = nullptr;
        uint16_t lpn_pin = 0;
        GPIO_TypeDef* interrupt_port = nullptr;
        uint16_t interrupt_pin = 0;

        Resolution resolution = Resolution::GRID_8X8;
        RangingMode ranging_mode = RangingMode::CONTINUOUS;
        uint8_t ranging_frequency_hz = 15;
        uint32_t integration_time_ms = 0;
        uint32_t vhv_repeat_count = 0;
        uint32_t read_failure_threshold = VL53L5CX_DEFAULT_FAILURE_THRESHOLD;
    };

    struct Calibration {
        std::array<uint8_t, VL53L5CX_XTALK_BUFFER_SIZE> crosstalk_data{};
        uint32_t margin_kcps_per_spad = 50;
        bool valid = false;
    };

    struct CalibrationSettings {
        uint16_t target_distance_mm = 600;
        uint16_t target_reflectance_percent = 3;
        uint8_t sample_count = 4;
    };

    explicit VL53L5CX(const char* name);
    VL53L5CX(const char* name, const Config& config, const Calibration* calibration = nullptr);
    ~VL53L5CX() override;

    // Config and calibration objects must remain valid; config must also remain unchanged.
    bool init(const Config& config, const Calibration* calibration = nullptr);
    bool start();
    bool stop();
    bool reset();
    bool checkConnection();

    bool setCalibration(const Calibration& calibration);
    // The output calibration must remain valid if it is retained by the driver.
    bool calibrateCrosstalk(const CalibrationSettings& settings, Calibration& calibration);
    bool getCalibration(Calibration& calibration);

    TofGeometry getGeometry() const override;
    bool isSampleReady() override;
    bool readSample(TofSample& sample) override;

    uint32_t getReadFailures() const;

   private:
    const Config* config = nullptr;
    const Calibration* calibration = nullptr;
    VL53L5CX_Configuration* device = nullptr;
    size_t device_slot = VL53L5CX_MAX_INSTANCES;

    uint32_t consecutive_read_failures = 0;
    uint8_t last_read_status = VL53L5CX_STATUS_OK;

    bool lpn_initialized = false;
    bool interrupt_initialized = false;
    bool interrupt_registered = false;
    volatile bool interrupt_pending = false;
    bool read_retry_pending = false;
    bool sensor_initialized = false;

    mutable osMutexId_t mutex_id = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};

    bool acquireDeviceContext();
    void releaseDeviceContext();

    bool initMutex();
    bool initGpios();
    bool initializeSensor();
    bool configureSensor(bool start_ranging);
    bool validateConfig(const Config& config) const;

    void releaseInterrupt();
    bool applyCalibration();
    bool setEnabled(bool enabled);
    bool probeAtAddress(uint8_t address);

    bool lock() const;
    void unlock() const;

    void handleReadResult(bool success, uint8_t status = VL53L5CX_STATUS_OK);
    void logApiError(const char* operation, uint8_t status) const;
    static void handleInterrupt(GPIO_TypeDef* port, uint16_t pin, void* context);
};
