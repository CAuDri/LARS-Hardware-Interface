/**
 * @file tof.hpp
 *
 * @brief Generic Time-of-Flight sensor driver interface
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "driver.hpp"

constexpr size_t TOF_MAX_ZONES = 64;
constexpr size_t TOF_MAX_SENSORS = 4;

/**
 * @brief Rectangular scene geometry shared by ToF drivers and publishers.
 *
 * Samples are row-major from the scene's top-left to bottom-right as viewed
 * along the sensor's optical axis. A single-zone sensor reports a 1x1 grid.
 */
struct TofGeometry {
    uint8_t rows = 1;
    uint8_t columns = 1;
    /** Full angular extent used to project axial depths into XYZ coordinates. */
    float horizontal_fov_rad = 0.0F;
    float vertical_fov_rad = 0.0F;
};

/**
 * @brief Fixed-capacity generic ToF sample.
 *
 * Only the first zone_count entries are valid. Drivers normalize native zone
 * ordering and quality rules while retaining native status for diagnostics.
 */
struct TofSample {
    /** Target depth perpendicular to the sensor plane (optical z-axis). */
    std::array<int16_t, TOF_MAX_ZONES> distance_mm{};
    /** Driver-normalized validity mask used by generic publishers. */
    std::array<bool, TOF_MAX_ZONES> valid{};
    /** Sensor-native target status retained for diagnostics. */
    std::array<uint8_t, TOF_MAX_ZONES> status{};
    /** Estimated target reflectance when supplied by the sensor. */
    std::array<uint8_t, TOF_MAX_ZONES> reflectance_percent{};
    /** Local monotonic timestamp captured after the frame transfer. */
    uint32_t timestamp_ms = 0;
    /** Sensor-internal temperature associated with the frame. */
    int8_t temperature_c = 0;
    /** Number of populated entries in each zone array. */
    uint8_t zone_count = 0;
};

class TofDriver : public Driver {
   public:
    /**
     * @brief Construct a generic ToF driver base.
     * @param name Stable driver name used in logs and diagnostics.
     */
    explicit TofDriver(const char* name) : Driver(name) {}

    /** @brief Destroy a generic ToF driver base. */
    ~TofDriver() override = default;

    /**
     * @brief Get the scene-oriented grid dimensions and full field of view.
     * @return Geometry describing the ordering and angular extent of samples.
     */
    virtual TofGeometry getGeometry() const = 0;

    /**
     * @brief Check whether a new sample can be read without waiting.
     * @return true when readSample() should be called.
     */
    virtual bool isSampleReady() = 0;

    /**
     * @brief Read one complete sample using the ordering from getGeometry().
     * @param sample Destination for generic ranges and quality information.
     * @return true when the complete sample was read.
     */
    virtual bool readSample(TofSample& sample) = 0;
};
