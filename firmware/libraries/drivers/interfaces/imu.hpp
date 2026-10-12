/**
 * @file imu.hpp
 *
 * @brief CAuDri - Generic IMU driver interface
 */
#pragma once

#include <cstdint>

#include "driver.hpp"

struct ImuSample {
    float acceleration_mps2[3]{};
    float angular_velocity_radps[3]{};
    float magnetic_field_ut[3]{};
    float orientation_xyzw[4]{};
    uint32_t timestamp_ms = 0;
};

class ImuDriver : public Driver {
   public:
    explicit ImuDriver(const char* name) : Driver(name) {}
    ~ImuDriver() override = default;

    virtual bool isSampleReady() const { return true; }
    virtual bool readSample(ImuSample& sample) = 0;
};
