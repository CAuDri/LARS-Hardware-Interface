/**
 * @file tof_publisher.hpp
 *
 * @brief Generic micro-ROS Time-of-Flight point cloud publisher
 */
#pragma once

#include <cmsis_os2.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "client.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "tof.hpp"

using ros::BasePublisher;
using ros::Client;
using ros::EntityState;
using ros::Node;
using ros::Publisher;

constexpr size_t TOF_PUBLISHER_MAX_SENSORS = TOF_MAX_SENSORS;
constexpr uint32_t TOF_PUBLISHER_THREAD_STACK_SIZE = 3072;
constexpr uint32_t TOF_PUBLISHER_DEFAULT_PERIOD_MS = 10;
constexpr uint32_t TOF_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD = 5;

class TofPublisher : public Node {
   public:
    /**
     * @brief Configuration for the ToF publisher node.
     * 
     * @param publish_period_ms The period in milliseconds at which to publish ToF samples.
     * @param read_failure_threshold The number of consecutive read failures before reporting an error.
     * @param thread_priority The priority of the publisher thread (default: osPriorityNormal).
     * @param publisher_config Configuration for the underlying ROS publisher
     */
    struct Config {
        uint32_t publish_period_ms = TOF_PUBLISHER_DEFAULT_PERIOD_MS;
        uint32_t read_failure_threshold = TOF_PUBLISHER_DEFAULT_READ_FAILURE_THRESHOLD;
        osPriority_t thread_priority = osPriorityNormal;
        BasePublisher::Config publisher_config{false, 0};
    };

    TofPublisher();
    /** @brief Destroy the publisher node. Its firmware lifetime is expected to be static. */
    ~TofPublisher() = default;
    TofPublisher(const TofPublisher&) = delete;
    TofPublisher& operator=(const TofPublisher&) = delete;

    rcl_ret_t init(Client& client, const Config& config);
    bool registerTof(TofDriver& tof, const char* topic, const char* frame_id);
    rcl_ret_t start();
    size_t getTofCount() const;

   private:
    struct TofSlot {
        TofDriver* tof = nullptr;
        const char* topic = nullptr;
        const char* frame_id = nullptr;
        TofGeometry geometry{};
        float horizontal_scale = 0.0F;
        float vertical_scale = 0.0F;
        Publisher<sensor_msgs__msg__PointCloud2> publisher{};
        sensor_msgs__msg__PointCloud2 message{};
        uint32_t consecutive_read_failures = 0;
        bool publisher_initialized = false;
        bool read_failure_reported = false;
        bool publish_failure_reported = false;
    };

    Client* client = nullptr;
    Config config{};
    std::array<TofSlot, TOF_PUBLISHER_MAX_SENSORS> sensors{};
    size_t sensor_count = 0;
    bool started = false;

    osThreadId_t thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};

    rcl_ret_t initPublishers();

    void thread();

    void publishSensors();
    void publishSensor(TofSlot& slot);
    void fillPointCloud(TofSlot& slot, const TofSample& sample);

    void handleReadFailure(TofSlot& slot);
    void handleReadRecovery(TofSlot& slot);
    void handlePublishResult(TofSlot& slot, rcl_ret_t result);
};
