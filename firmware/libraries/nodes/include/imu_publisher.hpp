/**
 * @file imu_publisher.hpp
 *
 * @brief CAuDri - micro-ROS node for publishing IMU samples
 */
#pragma once

#include <cmsis_os2.h>
#include <sensor_msgs/msg/imu.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "imu.hpp"
#include "node.hpp"
#include "publisher.hpp"

using ros::BasePublisher;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Publisher;

constexpr size_t IMU_PUBLISHER_MAX_IMUS = 1;
constexpr uint32_t IMU_PUBLISHER_THREAD_STACK_SIZE = 1536;
constexpr uint32_t IMU_PUBLISHER_DEFAULT_PERIOD_MS = 20;
constexpr uint32_t IMU_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD = 5;
constexpr uint32_t IMU_PUBLISHER_DEFAULT_SAMPLE_READY_TIMEOUT_MS = 250;
constexpr uint32_t IMU_PUBLISHER_DEFAULT_RECOVERY_PROBE_INTERVAL_MS = 1000;

class ImuPublisher : public Node {
   public:
    /**
     * @brief Configuration for the IMU publisher node.
     * 
     * @param publish_period_ms The period in milliseconds at which to publish IMU samples.
     * @param read_failure_threshold The number of consecutive read failures before reporting an error.
     * @param sample_ready_timeout_ms The timeout in milliseconds to wait for a sample to be ready before reporting a read failure.
     * @param recovery_probe_interval_ms The interval in milliseconds to probe the IMU for recovery after a read failure.
     * @param thread_priority The priority of the publisher thread (default: osPriorityNormal).
     * @param publisher_config Configuration for the underlying ROS publisher
     */
    struct Config {
        uint32_t publish_period_ms = IMU_PUBLISHER_DEFAULT_PERIOD_MS;
        uint32_t read_failure_threshold = IMU_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD;
        uint32_t sample_ready_timeout_ms = IMU_PUBLISHER_DEFAULT_SAMPLE_READY_TIMEOUT_MS;
        uint32_t recovery_probe_interval_ms = IMU_PUBLISHER_DEFAULT_RECOVERY_PROBE_INTERVAL_MS;
        osPriority_t thread_priority = osPriorityNormal;
        BasePublisher::Config publisher_config{true, 0};
    };

    ImuPublisher();
    ~ImuPublisher() = default;
    ImuPublisher(const ImuPublisher&) = delete;
    ImuPublisher& operator=(const ImuPublisher&) = delete;

    rcl_ret_t init(Client& client, const Config& config);
    bool registerImu(ImuDriver& imu, const char* imu_topic, const char* frame_id);
    rcl_ret_t start();

    size_t getImuCount() const;

   private:
    struct ImuSlot {
        ImuDriver* imu = nullptr;
        const char* imu_topic = nullptr;
        const char* frame_id = nullptr;

        Publisher<sensor_msgs__msg__Imu> imu_publisher{};
        sensor_msgs__msg__Imu imu_message{};

        uint32_t consecutive_read_failures = 0;
        uint32_t last_sample_ms = 0;
        uint32_t last_recovery_probe_ms = 0;

        bool publishers_initialized = false;
        bool readiness_bypassed = false;
        bool read_failure_reported = false;
        bool imu_publish_failure_reported = false;
    };

    Client* client = nullptr;
    Config config{};
    std::array<ImuSlot, IMU_PUBLISHER_MAX_IMUS> imus{};
    size_t imu_count = 0;
    bool started = false;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    std::array<uint32_t, IMU_PUBLISHER_THREAD_STACK_SIZE / sizeof(uint32_t)> thread_stack{};

    rcl_ret_t initPublishers();

    void thread();

    void publishImus(uint32_t now_ms);
    void publishImu(ImuSlot& slot, uint32_t now_ms);

    bool shouldProbeImu(const ImuSlot& slot, uint32_t now_ms) const;
    void fillMessages(ImuSlot& slot, const ImuSample& sample);

    void handleReadFailure(ImuSlot& slot, uint32_t now_ms);
    void handleReadRecovery(ImuSlot& slot);
    void handleImuPublishResult(ImuSlot& slot, rcl_ret_t result);
};
