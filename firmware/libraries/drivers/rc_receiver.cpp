/**
 * @file rc_receiver.cpp
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
#include "rc_receiver.hpp"

#include "logger.h"

using namespace crsf;

RCReceiver* RCReceiver::rx_callback_instance = nullptr;

/**
 * @brief Construct a new RCReceiver object for later initialization
 */
RCReceiver::RCReceiver() : Driver("rc_receiver") {}

/**
 * @brief Construct a new RCReceiver object with the given configuration
 */
RCReceiver::RCReceiver(const Config& config) : Driver("rc_receiver") { init(config); }

RCReceiver::~RCReceiver() {
    osThreadTerminate(receiver_thread);
    setState(State::UNINITIALIZED);
}

/**
 * @brief Initialize the RCReceiver driver with the given configuration
 *
 * @return true if initialization was successful
 */
bool RCReceiver::init(const Config& config) {
    if (state != State::UNINITIALIZED) {
        return false;
    }

    if (config.huart == nullptr) {
        LogError("RC Receiver: Invalid UART handle");
        setState(State::ERROR);
        return false;
    }

    this->config = &config;

    if (!initUART()) {
        setState(State::ERROR);
        return false;
    }

    event_attributes.name = "RC Connection Event";
    event_attributes.cb_mem = &event_control_block;
    event_attributes.cb_size = sizeof(event_control_block);

    connection_event = osEventFlagsNew(&event_attributes);
    if (connection_event == nullptr) {
        LogError("RC Receiver: Failed to create connection event flags");
        setState(State::ERROR);
        return false;
    }

    mutex_attributes.name = "RC Channel Mutex";
    mutex_attributes.cb_mem = &mutex_control_block;
    mutex_attributes.cb_size = sizeof(mutex_control_block);

    channel_mutex = osMutexNew(&mutex_attributes);
    if (channel_mutex == nullptr) {
        LogError("RC Receiver: Failed to create channel mutex");
        setState(State::ERROR);
        return false;
    }

    thread_attributes.name = "RC Receiver";
    thread_attributes.priority = config.task_priority;
    thread_attributes.stack_mem = &thread_stack;
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    receiver_thread = osThreadNew(
        // Helper function for non-static member thread function
        [](void* arg) -> void {
            auto* obj = static_cast<RCReceiver*>(arg);
            obj->receiverThread(arg);
        },
        this,
        &thread_attributes);
    if (receiver_thread == nullptr) {
        LogError("RC Receiver: Failed to create receiver thread");
        setState(State::ERROR);
        return false;
    }

    LogInfo("RC Receiver: Driver initialized");
    setState(State::INITIALIZED);
    return true;
}

/**
 * @brief Start the RCReceiver driver
 *
 * The driver needs to be initialized. Will start reception and processing of RC data.
 *
 * @return true if the driver was started successfully
 */
bool RCReceiver::start() {
    if (state != State::INITIALIZED) {
        LogWarning("RC Receiver: Cannot start, driver not initialized");
        return false;
    }

    osThreadFlagsSet(receiver_thread, RC_START_THREAD_FLAG);
    return true;
}

/**
 * @brief Check if the receiver is currently connected to a transmitter
 *
 * @return true if connected, false otherwise
 */
bool RCReceiver::isConnected() const { return is_connected; }

/**
 * @brief Wait for the receiver to be connected to a transmitter
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = wait indefinitely)
 * @return true if the receiver is connected, false if the timeout was reached or an error occurred
 */
bool RCReceiver::waitForConnect(uint32_t timeout_ms) const {
    if (connection_event == nullptr) {
        return false;
    }
    uint32_t flags = osEventFlagsWait(connection_event, CONNECTED_EVENT_FLAG, osFlagsWaitAny, timeout_ms);
    return (flags & CONNECTED_EVENT_FLAG) != 0;
}

/**
 * @brief Wait for the receiver to be disconnected from the transmitter
 *
 * @param timeout_ms Maximum time to wait in milliseconds (0 = wait indefinitely)
 * @return true if the receiver is disconnected, false if the timeout was reached or an error occurred
 */
bool RCReceiver::waitForDisconnect(uint32_t timeout_ms) const {
    if (connection_event == nullptr) {
        return false;
    }
    uint32_t flags = osEventFlagsWait(connection_event, DISCONNECTED_EVENT_FLAG, osFlagsWaitAny, timeout_ms);
    return (flags & DISCONNECTED_EVENT_FLAG) != 0;
}

/**
 * @brief Get the latest received channel data
 *
 * The channel data is considered valid if it was updated within the last CHANNEL_DATA_VALIDITY_MS milliseconds.
 *
 * @param channels Reference to a ChannelData array to store the received channel values
 * @return true if valid channel data was retrieved, false otherwise
 */
bool RCReceiver::getChannelData(crsf::ChannelData& channels) const {
    if (state != State::RUNNING) {
        return false;
    }
    if (osKernelGetTickCount() - channel_update_ms > CHANNEL_DATA_VALIDITY_MS) {
        LogWarning("RC Receiver: Channel data is older than %lu ms", CHANNEL_DATA_VALIDITY_MS);
        return false;
    }
    if (osMutexAcquire(channel_mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != osOK) {
        LogWarning("RC Receiver: Failed to acquire channel mutex while getting channel data");
        return false;
    }

    channels = channel_data;
    osMutexRelease(channel_mutex);
    return true;
}

/**
 * @brief Get the latest received link statistics
 *
 * The statistics data is considered valid if it was updated within the last STATISTICS_DATA_VALIDITY_MS milliseconds.
 *
 * @param stats Reference to a LinkStatistics struct to store the received statistics
 * @return true if valid statistics data was retrieved, false otherwise
 */
bool RCReceiver::getLinkStatistics(crsf::LinkStatistics& stats) const {
    if (state != State::RUNNING) {
        return false;
    }
    if (osKernelGetTickCount() - statistics_update_ms > STATISTICS_DATA_VALIDITY_MS) {
        LogWarning("RC Receiver: Link statistics data is older than %lu ms", STATISTICS_DATA_VALIDITY_MS);
        return false;
    }
    stats = statistics;
    return true;
}

/**
 * @brief Register a callback function for a specific channel
 *
 * The callback will be called whenever a new value is received for the specified channel.
 * If a callback is already registered for the channel, it will be overwritten.
 *
 * @param channel Channel number (0-15)
 * @param callback Function pointer to the callback function
 * @param on_change If true, the callback will only be called when the channel value changes, otherwise it will be called on every update
 * @return true if the callback was registered successfully, false otherwise
 */
bool RCReceiver::registerChannelCallback(size_t channel, ChannelCallback callback, bool on_change) {
    if (channel >= channel_callbacks.size()) {
        LogError("RC Receiver: Invalid channel number %zu for callback registration, max: %zu",
                 channel,
                 channel_callbacks.size() - 1);
        return false;
    }
    if (!callback) {
        LogError("RC Receiver: Callback function is null for channel %zu", channel);
        return false;
    }
    if (!channel_callbacks[channel]) {
        LogWarning("RC Receiver: Overwriting existing callback for channel %zu", channel);
    }
    channel_callbacks[channel] = callback;
    callback_on_change[channel] = on_change;
    return true;
}

/**
 * @brief Initialize the UART interface for communication with the receiver
 */
bool RCReceiver::initUART() {
    // Check if RX DMA is linked to the UART and enabled
    if (config->huart->hdmarx == nullptr || config->huart->hdmarx->Instance == nullptr) {
        LogError("RC Receiver: UART RX DMA not configured");
        return false;
    }

    // Set the configured baud rate and reinitialize the UART
    config->huart->Init.BaudRate = config->baud_rate;
    if (HAL_UART_Init(config->huart) != HAL_OK) {
        LogError("RC Receiver: Failed to (re)initialize UART");
        return false;
    }

    // For now we will only support a single instance of the RCReceiver driver
    // In the future we could use a map of UART handles to RCReceiver instances if needed
    if (rx_callback_instance != nullptr) {
        LogError("RC Receiver: Only a single driver instance is supported");
        return false;
    }
    rx_callback_instance = this;

    // Register the RX event callback for IDLE line and transfer complete events
    HAL_StatusTypeDef status;
    status = HAL_UART_RegisterRxEventCallback(config->huart, rxCallback);
    if (status != HAL_OK) {
        LogError("RC Receiver: Failed to register RX event callback, status: %d", status);
        return false;
    }

    // Register the error callback for any kind of UART errors
    status = HAL_UART_RegisterCallback(config->huart, HAL_UART_ERROR_CB_ID, errorCallback);
    if (status != HAL_OK) {
        LogError("RC Receiver: Failed to register error callback, status: %d", status);
        return false;
    }

    if (!enableUARTInterrupts()) {
        setState(State::ERROR);
        return false;
    }
    return true;
}

/**
 * @brief Enable UART interrupts for receiving data via DMA
 */
bool RCReceiver::enableUARTInterrupts() {
    // Disable the DMA half-complete interrupt
    __HAL_UART_DISABLE_IT(config->huart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(config->huart, UART_IT_TC);

    HAL_StatusTypeDef status;
    status = HAL_UARTEx_ReceiveToIdle_DMA(config->huart, rx_buffer.data(), rx_buffer.size());
    if (status != HAL_OK) {
        LogError("RC Receiver: Failed to enable UART DMA reception, status: %d", status);
        return false;
    }
    return true;
}

/**
 * @brief Publish the connection status to any waiting threads using OS events
 */
bool RCReceiver::publishConnectionStatus(bool connected) {
    if (connection_event == nullptr) {
        return false;
    }
    if (connected) {
        this->is_connected = true;
        osEventFlagsSet(connection_event, CONNECTED_EVENT_FLAG);
    } else {
        this->is_connected = false;
        osEventFlagsSet(connection_event, DISCONNECTED_EVENT_FLAG);
    }
    return true;
}

/**
 * @brief Thread function for processing incoming RC data
 */
void RCReceiver::receiverThread(void* arg) {
    osThreadFlagsWait(RC_START_THREAD_FLAG, osFlagsWaitAny, osWaitForever);
    LogInfo("RC Receiver: Started receiver thread");
    setState(State::RUNNING);

    while (state == State::RUNNING) {
        last_frame_parsed = true;
        uint32_t flags = osThreadFlagsWait(RC_RX_COMPLETE_THREAD_FLAG, osFlagsWaitAny, config->timeout);

        if (flags == osFlagsErrorTimeout) {
            LogWarning("RC Receiver: Timeout waiting for RX complete flag");
            // Reenable UART interrupts as a recovery measure
            if (config->huart->gState == HAL_UART_STATE_READY) {
                enableUARTInterrupts();
            }
            continue;
        }
        if (flags != RC_RX_COMPLETE_THREAD_FLAG) {
            LogWarning("RC Receiver: Unexpected error waiting for RX complete flag, flags: 0x%08lX", flags);
            continue;
        }

        if (last_frame_parsed || frame_size == 0) {
            LogWarning("RC Receiver: No new frame to process");
            continue;
        }

        ParseResult result;
        if (!parseFrame(frame_buffer.data(), frame_size, result)) {
            LogWarning("RC Receiver: Failed to parse frame");
            continue;
        }
        if (!result.valid) {
            LogWarning("RC Receiver: Invalid frame received");
            continue;
        }

        // Process known frame types
        switch (result.frame_type) {
            case FrameType::RC_CHANNELS_PACKED: {
                if (result.length < RC_CHANNELS_PAYLOAD_SIZE) {
                    LogWarning("RC Receiver: RC_CHANNELS_PACKED frame too short");
                    break;
                }

                ChannelData channels;
                parseChannelData(result.payload.data(), channels);

                for (size_t i = 0; i < channels.size(); ++i) {
                    if (channel_callbacks[i]) {
                        if (callback_on_change[i] && channels[i] == channel_data[i]) {
                            continue;  // Skip callback if value hasn't changed
                        }
                        channel_callbacks[i](channels[i]);
                    }
                }

                if (osMutexAcquire(channel_mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != osOK) {
                    LogWarning(
                        "RC Receiver: Failed to acquire mutex while processing channel data");
                    break;
                }
                this->channel_data = channels;
                this->channel_update_ms = osKernelGetTickCount();
                osMutexRelease(channel_mutex);
                break;
            }
            case FrameType::LINK_STATISTICS: {
                if (result.length < LINK_STATISTICS_PAYLOAD_SIZE) {
                    LogWarning("RC Receiver: LINK_STATISTICS frame too short");
                    break;
                }

                LinkStatistics stats;
                parseLinkStatistics(result.payload.data(), stats);

                this->statistics = stats;
                this->statistics_update_ms = osKernelGetTickCount();
                break;
            }
            case FrameType::HEARTBEAT:
                // Heartbeat frame received, can be used to monitor connection status
                LogDebug("RC Receiver: Heartbeat frame received");
                break;
            case FrameType::PING:
                // As per CRSF spec, receivers should respond with a DEVICE_INFO frame
                // TODO: Implement sending a DEVICE_INFO frame in response
                LogDebug("RC Receiver: Ping frame received");
                break;
            default:
                LogDebug("RC Receiver: Received unsupported frame type 0x%02X", result.frame_type);
                break;
        }
    }
    LogInfo("RC Receiver: Exiting receiver thread");
    osThreadExit();
}

/**
 * @brief Static callback function for UART RX events
 *
 * This function is called by the HAL library when data is received via UART.
 * It forwards the call to the instance method of the RCReceiver class.
 *
 * @note This implementation currently supports only a single instance of RCReceiver.
 */
void RCReceiver::rxCallback(UART_HandleTypeDef* huart, uint16_t pos) {
    // For now we will only support a single instance of the RCReceiver driver
    // Sadly, there is no way to store user data in the HAL UART handle
    // In the future, a map of UART handles to RCReceiver instances can be used if needed
    if (rx_callback_instance != nullptr) {
        rx_callback_instance->onReceive(pos);
    }
}

/**
 * @brief Instance method called when data is received via UART
 *
 * This method processes the received data and notifies the receiver thread.
 *
 * @note This implementation currently supports only a single instance of RCReceiver.
 */
void RCReceiver::onReceive(uint16_t pos) {
    // TODO: Check what triggered the RX callback (HAL_UARTEx_GetRxEventType)

    // Copy the received data from the DMA buffer to our frame buffer
    // If we are still processing the last frame, we will drop the new data for now
    if (!last_frame_parsed) {
        LogDebug("RC Receiver: Previous frame not yet processed, dropping received data");
        osThreadFlagsSet(receiver_thread, RC_RX_COMPLETE_THREAD_FLAG);
        return;
    }

    if (pos == 0) {
        LogDebug("RC Receiver: Received zero bytes, ignoring");
        return;
    }

    if (pos > frame_buffer.size()) {
        LogDebug("RC Receiver: Received data exceeds frame buffer size, truncating data");
        pos = frame_buffer.size();
    }

    // Copy the received data to the frame buffer for processing
    this->frame_size = pos;
    std::copy(rx_buffer.begin(), rx_buffer.begin() + frame_size, frame_buffer.begin());
    last_frame_parsed = false;

    // Notify the receiver thread that new data is available
    osThreadFlagsSet(receiver_thread, RC_RX_COMPLETE_THREAD_FLAG);

    // Reenable DMA reception for the next frame
    rx_callback_instance->enableUARTInterrupts();
}

/**
 * @brief Static callback function for UART error events
 *
 * @note This implementation currently supports only a single instance of RCReceiver.
 */
void RCReceiver::errorCallback(UART_HandleTypeDef* huart) {
    // For now we will only support a single instance of the RCReceiver driver
    // Sadly, there is no way to store user data in the HAL UART handle
    // In the future, a map of UART handles to RCReceiver instances can be used if needed
    if (rx_callback_instance != nullptr) {
        uint32_t error = huart->ErrorCode;
        switch (error) {
            case HAL_UART_ERROR_NONE:
                // No error
                break;
            case HAL_UART_ERROR_PE:
                LogError("RC Receiver: UART Parity Error occurred");
                break;
            case HAL_UART_ERROR_NE:
                LogError("RC Receiver: UART Noise Error occurred");
                break;
            case HAL_UART_ERROR_FE:
                LogError("RC Receiver: UART Framing Error occurred");
                break;
            case HAL_UART_ERROR_ORE:
                LogError("RC Receiver: UART Overrun Error occurred");
                break;
            case HAL_UART_ERROR_DMA:
                LogError("RC Receiver: UART DMA Error occurred");
                break;
            default:
                LogError("RC Receiver: UART Unknown Error occurred, code: 0x%08lX", error);
                break;
        }
    }

    // Reenable DMA reception after an error
    rx_callback_instance->enableUARTInterrupts();
}