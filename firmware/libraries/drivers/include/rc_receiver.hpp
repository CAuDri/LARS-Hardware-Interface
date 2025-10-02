/**
 * @file rc_receiver.hpp
 *
 * @brief CAuDri - RC Receiver Driver using the CRSF Protocol
 *
 * This driver implements the communication with a remote control transmitter using the CRSF (Crossfire) protocol.
 * CRSF is a low-latency, high-performance protocol commonly used in FPV drones and RC vehicles and is supported by many modern transmitters and receivers (TBS Crossfire, ExpressLRS, etc.).
 *
 * The driver handles the reception of RC channel data and link statistics (RSSI, SNR, etc.) from the receiver module.
 * A callback can be registered for each channel that will be called whenever a new value is received.
 *
 * Communication is done over a UART interface using a DMA stream for efficient data transfer.
 * The driver runs a dedicated FreeRTOS task for processing incoming data and managing the receiver state.
 */
#pragma once

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "callback_wrapper.hpp"
#include "crsf_protocol.hpp"
#include "driver.hpp"


// Stack size for the receiver thread in bytes
constexpr uint32_t RC_THREAD_STACK_SIZE = 1024;

// Time in ms after which channel data is considered stale
constexpr uint32_t CHANNEL_DATA_VALIDITY_MS = 100;
// Time in ms after which statistics data is considered stale
constexpr uint32_t STATISTICS_DATA_VALIDITY_MS = 1000;

// Thread flags for notifying the receiver thread
// Change if any conflicts arise
constexpr uint32_t RC_START_THREAD_FLAG = 0x01;
constexpr uint32_t RC_RX_COMPLETE_THREAD_FLAG = 0x02;

/**
 * @brief Driver for communicating with an RC receiver using the CRSF protocol
 */
class RCReceiver : public Driver {
   public:
    /**
     * @brief Configuration for the RCReceiver driver
     *
     * @param huart Pointer to a configured UART handle with DMA enabled (required)
     * @param timeout Time in milliseconds without valid RC data before considering the link lost
     * @param task_priority FreeRTOS task priority for the receiver thread
     */
    struct Config {
        UART_HandleTypeDef* huart = nullptr;

        uint32_t baud_rate = crsf::BAUD_RATE;
        uint32_t timeout = crsf::CONNECTION_TIMEOUT_MS;
        osPriority_t task_priority = osPriorityHigh;
    };

    /**
     * @brief Callback type for channel updates
     */
    using ChannelCallback = CallbackWrapper<void(uint16_t)>;

    RCReceiver();
    explicit RCReceiver(const Config& config);
    ~RCReceiver() override;

    bool init(const Config& config);
    bool start();

    bool isConnected() const;
    bool waitForConnect(uint32_t timeout_ms = 0) const;
    bool waitForDisconnect(uint32_t timeout_ms = 0) const;

    bool registerChannelCallback(size_t channel, ChannelCallback callback, bool on_change = true);
    bool getChannelData(crsf::ChannelData& channels) const;
    bool getLinkStatistics(crsf::LinkStatistics& stats) const;

   private:
    enum EventFlags : uint32_t { CONNECTED_EVENT_FLAG = 0x01, DISCONNECTED_EVENT_FLAG = 0x02 };

    const Config* config = nullptr;

    osThreadId_t receiver_thread = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[RC_THREAD_STACK_SIZE / 4]{};

    osEventFlagsId_t connection_event = nullptr;
    osEventFlagsAttr_t event_attributes{};
    StaticEventGroup_t event_control_block{};

    osMutexId_t channel_mutex = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};
    const size_t MUTEX_ACQUIRE_TIMEOUT_MS = 100;

    bool is_connected = false;

    // Stored instance for static UART callbacks
    static RCReceiver* rx_callback_instance;

    std::array<uint8_t, 256> rx_buffer{};                            // DMA reception buffer
    std::array<uint8_t, crsf::MAX_TOTAL_FRAME_SIZE> frame_buffer{};  // Buffer for the current frame being processed
    size_t frame_size = 0;          // Size of the current frame in the buffer
    bool last_frame_parsed = true;  // Whether the last received frame has been processed

    crsf::ChannelData channel_data{};   // Last received channel data
    uint32_t channel_update_ms = 0;     // Timestamp of the last channel update
    crsf::LinkStatistics statistics{};  // Last received link statistics
    uint32_t statistics_update_ms = 0;  // Timestamp of the last statistics update

    std::array<ChannelCallback, 16> channel_callbacks{};  // Callbacks for each of the 16 channels
    std::array<bool, 16> callback_on_change{};  // Whether to call the callback only on value change

    bool initUART();
    bool enableUARTInterrupts();
    bool publishConnectionStatus(bool connected);

    void receiverThread(void* arg);

    static void rxCallback(UART_HandleTypeDef* huart, uint16_t pos);
    static void errorCallback(UART_HandleTypeDef* huart);
    void onReceive(uint16_t pos);
};
