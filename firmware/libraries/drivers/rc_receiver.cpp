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

// #define LOG_VERBOSE

#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

RCReceiver* RCReceiver::rx_callback_instance = nullptr;

/**
 * @brief Construct a new RCReceiver object for later initialization
 */
RCReceiver::RCReceiver() : Driver("rc_receiver") {}

/**
 * @brief Construct a new RCReceiver object with the given configuration
 */
RCReceiver::RCReceiver(const Config& config) : Driver("rc_receiver") { init(config); }

/**
 * @brief Destruct the RCReceiver object and terminate the receiver thread
 */
RCReceiver::~RCReceiver() {
    osThreadTerminate(receiver_thread);
    publishConnectionState(false);
    setState(State::UNINITIALIZED);
}

/**
 * @brief Initialize the RCReceiver driver with the given configuration
 *
 * This will configure and initialize the UART connection and start the receiver thread.
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

    expected_message_interval_ms = 1000 / config.channel_data_rate_hz;
    if (expected_message_interval_ms == 0) {
        expected_message_interval_ms = 1;
    }
    LogDebug("RC Receiver: Expected message interval: %lu ms", expected_message_interval_ms);

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

    queue_attributes.name = "RC RX Queue";
    queue_attributes.cb_mem = &queue_control_block;
    queue_attributes.cb_size = sizeof(queue_control_block);
    queue_attributes.mq_mem = rx_queue_buffer.data();
    queue_attributes.mq_size = sizeof(rx_queue_buffer);

    rx_queue = osMessageQueueNew(RC_RX_QUEUE_LENGTH, sizeof(RXQueueMessage), &queue_attributes);
    if (rx_queue == nullptr) {
        LogError("RC Receiver: Failed to create RX message queue");
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
        // Helper function for using a non-static method as the thread entry point
        // The 'this' pointer is passed as the user argument to the lambda
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
 * The driver needs to be initialized. This will start reception and processing of RC data.
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
bool RCReceiver::isConnected() const { return connection_state == ConnectionState::CONNECTED; }

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
    if (connection_state == ConnectionState::CONNECTED) {
        return true;
    }
    uint32_t flags = osEventFlagsWait(connection_event, CONNECTED_EVENT_FLAG, osFlagsNoClear, timeout_ms);
    return (flags & CONNECTED_EVENT_FLAG);
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
    if (connection_state == ConnectionState::DISCONNECTED) {
        return true;
    }
    uint32_t flags = osEventFlagsWait(connection_event, DISCONNECTED_EVENT_FLAG, osFlagsNoClear, timeout_ms);
    return (flags & DISCONNECTED_EVENT_FLAG);
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
        LogError("RC Receiver: Callback function is null for channel %u", channel);
        return false;
    }
    if (channel_callbacks[channel]) {
        LogWarning("RC Receiver: Overwriting existing callback for channel %u", channel);
    }
    channel_callbacks[channel] = callback;
    callback_on_change[channel] = on_change;
    return true;
}

/**
 * @brief Get the latest received channel data
 *
 * The channel data is considered valid if it was updated within the last RC_CHANNEL_DATA_VALIDITY_MS milliseconds.
 *
 * @param channels Reference to a ChannelData array to store the received channel values
 * @return true if valid channel data was retrieved, false otherwise
 */
bool RCReceiver::getChannelData(crsf::ChannelData& channels) const {
    if (state != State::RUNNING) {
        return false;
    }
    if (osKernelGetTickCount() - channel_update_ms > RC_CHANNEL_DATA_VALIDITY_MS) {
        LogWarning("RC Receiver: Channel data is older than %lu ms", RC_CHANNEL_DATA_VALIDITY_MS);
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
 * The statistics data is considered valid if it was updated within the last RC_STATISTICS_DATA_VALIDITY_MS milliseconds.
 *
 * @param stats Reference to a LinkStatistics struct to store the received statistics
 * @return true if valid statistics data was retrieved, false otherwise
 */
bool RCReceiver::getLinkStatistics(crsf::LinkStatistics& stats) const {
    if (state != State::RUNNING) {
        return false;
    }
    if (osKernelGetTickCount() - statistics_update_ms > RC_STATISTICS_DATA_VALIDITY_MS) {
        LogWarning("RC Receiver: Link statistics data is older than %lu ms", RC_STATISTICS_DATA_VALIDITY_MS);
        return false;
    }
    stats = statistics;
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

    // Set the configured baud rate and reinitialize the UART peripheral
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
        LogError("RC Receiver: Failed to enable UART interrupts");
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
        LogVerbose("RC Receiver: Failed to enable UART DMA reception, status: %d", status);
        return false;
    }
    return true;
}

/**
 * @brief Publish the connection status to any waiting threads using OS events
 */
bool RCReceiver::publishConnectionState(bool connected) {
    if (connection_event == nullptr) {
        return false;
    }
    if (connected) {
        osEventFlagsSet(connection_event, CONNECTED_EVENT_FLAG);
    } else {
        osEventFlagsSet(connection_event, DISCONNECTED_EVENT_FLAG);
    }
    return true;
}

/**
 * @brief Handle a received CRSF message
 *
 * @param result Parsed CRSF message
 * @return true if the message was handled successfully, false otherwise
 */
bool RCReceiver::handleReceivedMessage(crsf::ParseResult& result) {
    if (!result.valid) {
        LogWarning("RC Receiver: Invalid frame received");
        return false;
    }

    switch (result.frame_type) {
        case FrameType::RC_CHANNELS_PACKED: {
            if (result.length < RC_CHANNELS_PAYLOAD_SIZE) {
                LogWarning("RC Receiver: RC_CHANNELS_PACKED frame too short");
                break;
            }

            ChannelData channels;
            parseChannelData(result.payload.data(), channels);

            if (connection_state != ConnectionState::CONNECTED) {
                LogVerbose("RC Receiver: Channel data received, ignore callbacks until connection is established");
                break;
            }

            // Invoke registered callbacks for each channel
            for (size_t i = 0; i < channels.size(); ++i) {
                if (channel_callbacks[i]) {
                    if (callback_on_change[i] && channels[i] == channel_data[i]) {
                        continue;  // Skip callback if value hasn't changed
                    }
                    channel_callbacks[i](channels[i]);
                }
            }

            if (osMutexAcquire(channel_mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != osOK) {
                LogWarning("RC Receiver: Failed to acquire mutex while processing channel data");
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
            return false;
    }

    return true;
}

/**
 * @brief Thread function for processing incoming RC data
 */
void RCReceiver::receiverThread(void* arg) {
    // Wait for the start signal from the main application
    uint32_t flags = osThreadFlagsWait(RC_START_THREAD_FLAG, osFlagsWaitAny, osWaitForever);
    if (!(flags & RC_START_THREAD_FLAG) || (flags & osFlagsError)) {
        LogError("RC Receiver: Error starting receiver thread, flags: 0x%08lX", flags);
        setState(State::ERROR);
        osDelay(osWaitForever);
    }

    LogInfo("RC Receiver: Driver started, connecting to receiver...");
    setState(State::RUNNING);
    publishConnectionState(false);

    // Some receivers may send some initial frames on power-up, we will wait a bit to avoid false timeouts
    osDelay(1000);

    const uint32_t MESSAGE_TIMEOUT_MS = expected_message_interval_ms * RC_MAX_MESSAGE_DELAY_FACTOR;

    uint32_t missed_frame_count = 0;
    uint32_t valid_frame_count = 0;

    uint32_t disconnect_count = 0;
    uint32_t disconnect_timestamp = 0;

    while (getState() == State::RUNNING) {
        // TODO:  Find out why sometimes the UART interrupts won't reenable properly
        // This is a temporary workaround to ensure reception continues
        if (config->huart->RxState == HAL_UART_STATE_READY) {
            enableUARTInterrupts();
        }

        uint32_t flags = osThreadFlagsGet();
        if (flags & RC_ERROR_THREAD_FLAG) {
            LogWarning("RC Receiver: Error flag set in receiver thread)");
            // TODO: Find out if UART error handling is needed here
            osThreadFlagsClear(RC_ERROR_THREAD_FLAG);
        }

        RXQueueMessage rx_message;
        ParseResult result;
        osStatus_t status;

        switch (connection_state) {
            case ConnectionState::UNKNOWN:  // Initial connection attempt, wait for any valid frame to be received
                status = osMessageQueueGet(rx_queue, &rx_message, nullptr, RC_INITIAL_CONNECTION_TIMEOUT_MS);
                if (status == osErrorTimeout) {
                    LogError("RC Receiver: Initial connection timeout after %lu ms, shutting down", RC_INITIAL_CONNECTION_TIMEOUT_MS);
                    setConnectionState(ConnectionState::DISCONNECTED);
                    setState(State::ERROR);
                    continue;
                } else if (status != osOK) {
                    LogError(
                        "RC Receiver: Error receiving from RX queue during initial connection, status: %d, shutting "
                        "down",
                        status);
                    setState(State::ERROR);
                    continue;
                }

                if (!parseFrame(rx_message.data, rx_message.size, result)) {
                    LogVerbose("RC Receiver: Failed to parse frame during initial connection");
                    continue;
                }
                if (result.frame_type != FrameType::RC_CHANNELS_PACKED) {
                    LogVerbose("RC Receiver: Ignoring non-channel frame during initial connection, type 0x%02X", result.frame_type);
                    continue;
                }

                // A valid channel data frame was received and we will check if the connection stays up
                LogDebug("RC Receiver: Initial frame received, establishing connection");
                setConnectionState(ConnectionState::CONNECTING);
                break;

            case ConnectionState::CONNECTING:  // No messages will be handled until a stable connection is established
                if (missed_frame_count >= RC_MAX_DROPPED_FRAMES) {
                    LogWarning("RC Receiver: Missed too many frames during connection attempt");
                    missed_frame_count = 0;
                    valid_frame_count = 0;
                    setConnectionState(ConnectionState::DISCONNECTED);
                }

                // Before considering the connection established, a few valid frames need to be received consecutively
                status = osMessageQueueGet(rx_queue, &rx_message, nullptr, MESSAGE_TIMEOUT_MS);
                if (status == osErrorTimeout) {
                    missed_frame_count++;
                    valid_frame_count = 0;
                    LogVerbose("RC Receiver: Timeout waiting for frame while connecting (%lu/%u)", missed_frame_count, RC_MAX_DROPPED_FRAMES);
                    break;
                } else if (status != osOK) {
                    missed_frame_count++;
                    valid_frame_count = 0;
                    LogError("RC Receiver: Error receiving from RX queue during connection attempt, status: %d", status);
                    break;
                }

                if (!parseFrame(rx_message.data, rx_message.size, result)) {
                    missed_frame_count++;
                    valid_frame_count = 0;
                    LogVerbose("RC Receiver: Failed to parse frame during connection attempt (%lu/%u)", missed_frame_count, RC_MAX_DROPPED_FRAMES);
                    break;
                }

                // The missed frame count will not be reset, to avoid endless connection attempts if the link is bad
                valid_frame_count++;
                if (valid_frame_count >= RC_MIN_GOOD_FRAMES) {
                    LogSuccess("RC Receiver: Connection established");
                    setConnectionState(ConnectionState::CONNECTED);
                    publishConnectionState(true);
                    missed_frame_count = 0;
                    valid_frame_count = 0;
                }
                break;

            case ConnectionState::CONNECTED:  // Normal operation, process incoming frames and monitor connection state
                if (missed_frame_count >= RC_MAX_DROPPED_FRAMES) {
                    // The transceiver has probably gone out of range or powered off
                    // We will notify the rest of the system and try to reconnect
                    LogWarning("RC Receiver: Missed too many frames, considering link lost");
                    setConnectionState(ConnectionState::DISCONNECTED);
                    publishConnectionState(false);
                    missed_frame_count = 0;
                    valid_frame_count = 0;

                    // Invalidate channel data
                    channel_data.fill(0);  
                    channel_update_ms = 0;
                    break;
                }

                status = osMessageQueueGet(rx_queue, &rx_message, nullptr, MESSAGE_TIMEOUT_MS);
                if (status == osErrorTimeout) {
                    missed_frame_count++;
                    LogVerbose("RC Receiver: Timeout waiting for frame (%lu/%u)", missed_frame_count, RC_MAX_DROPPED_FRAMES);
                    break;
                } else if (status != osOK) {
                    missed_frame_count++;
                    LogWarning("RC Receiver: Error receiving from RX queue, status: %d", status);
                    break;
                }

                if (!parseFrame(rx_message.data, rx_message.size, result)) {
                    missed_frame_count++;
                    LogVerbose("RC Receiver: Failed to parse frame (%lu/%u)", missed_frame_count, RC_MAX_DROPPED_FRAMES);
                    break;
                }

                // Only in the CONNECTED state will messages be processed and the respective callbacks called
                if (!handleReceivedMessage(result)) {
                    LogVerbose("RC Receiver: Failed to handle received message");
                    break;
                }
                missed_frame_count = 0;
                break;

            case ConnectionState::DISCONNECTED:  // Connection lost, one last reconnect will be attempted before entering the ERROR state
                // To prevent endless reconnect attempts the total number of disconnects is limited
                if (disconnect_count >= RC_MAX_DISCONNECT_ATTEMPTS) {
                    LogError("RC Receiver: Maximum reconnect attempts reached (%lu), shutting down", RC_MAX_DISCONNECT_ATTEMPTS);
                    setState(State::ERROR);
                    continue;
                }
                if (disconnect_timestamp == 0) {  // First attempt after disconnect
                    disconnect_timestamp = osKernelGetTickCount();
                    LogWarning("RC Receiver: Trying to reconnect to the transmitter...");
                } else if ((osKernelGetTickCount() - disconnect_timestamp) >= RC_RECONNECT_TIMEOUT_MS) {
                    LogError(
                        "RC Receiver: Reconnect timeout after %lu ms with multiple invalid frames received, shutting "
                        "down",
                        RC_RECONNECT_TIMEOUT_MS);
                    setState(State::ERROR);
                    continue;
                }

                status = osMessageQueueGet(rx_queue, &rx_message, nullptr, RC_RECONNECT_TIMEOUT_MS);
                if (status == osErrorTimeout) {
                    LogError("RC Receiver: Reconnect was unsuccessful after %lu ms, shutting down", RC_RECONNECT_TIMEOUT_MS);
                    setState(State::ERROR);
                    continue;
                } else if (status != osOK) {
                    LogError("RC Receiver: Error receiving from RX queue during reconnect, status: %d, shutting down", status);
                    setState(State::ERROR);
                    continue;
                }

                if (!parseFrame(rx_message.data, rx_message.size, result)) {
                    LogVerbose("RC Receiver: Failed to parse frame during reconnect");
                    break;  // Will try again until timeout is reached
                }

                // A valid frame was received, try to re-establish the connection
                LogDebug("RC Receiver: Frame received during reconnect, re-establishing connection");
                disconnect_count++;
                disconnect_timestamp = 0;
                setConnectionState(ConnectionState::CONNECTING);
                break;

            default:
                break;
        }
    }
    LogError("RC Receiver: Receiver thread exiting due to error state");
    publishConnectionState(false);
    setConnectionState(ConnectionState::DISCONNECTED);
    osDelay(osWaitForever);
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
    if (receiver_thread == nullptr) {
        return;
    }

    // As long as the driver has not been started. the connection state is unknown
    // We don't want to fill up the RX queue in the meantime
    if (getState() != State::RUNNING) {
        return;
    }

    // TODO: Check what triggered the RX callback (HAL_UARTEx_GetRxEventType)
    auto event_type = HAL_UARTEx_GetRxEventType(config->huart);
    switch (event_type) {
        case HAL_UART_RXEVENT_HT:  // Half Transfer
            LogVerbose("RC Receiver: UART RX Half Transfer event");
            break;
        case HAL_UART_RXEVENT_TC:  // Transfer Complete
            LogVerbose("RC Receiver: UART RX Transfer Complete event");
            break;
        case HAL_UART_RXEVENT_IDLE:  // Idle Line Detected
            LogVerbose("RC Receiver: UART RX Idle Line Detected event");
            break;
        default:
            LogVerbose("RC Receiver: UART RX Unknown event type %lu", event_type);
            break;
    }

    if (pos == 0) {
        LogDebug("RC Receiver: Received zero bytes, ignoring");
        return;
    }
    if (pos > crsf::MAX_TOTAL_FRAME_SIZE) {
        LogWarning("RC Receiver: Received frame size %u exceeds maximum %u, dropping frame", pos, crsf::MAX_TOTAL_FRAME_SIZE);
        return;
    }

    // Push the received data to the RX queue for processing by the receiver thread
    // If the queue is full, the frame will be dropped and an error will be logged
    RXQueueMessage message;
    std::copy(rx_buffer.begin(), rx_buffer.begin() + pos, message.data);
    message.size = pos;
    message.timestamp = osKernelGetTickCount();
    if (osMessageQueuePut(rx_queue, &message, 0, 0) != osOK) {
        LogVerbose("RC Receiver: RX queue full, dropping received frame");
        error_count++;
        last_error_timestamp = message.timestamp;
        return;
    }
}

/**
 * @brief Static callback function for UART error events
 * 
 * Currently only used for debugging purposes since errors are handled in the receiver thread and will be detected due to missing frames.
 *
 * @note This implementation currently supports only a single instance of RCReceiver.
 */
void RCReceiver::errorCallback(UART_HandleTypeDef* huart) {
    // For now we will only support a single instance of the RCReceiver driver
    auto instance = rx_callback_instance;
    if (instance == nullptr) {
        return;
    }

    if (instance->getState() != State::RUNNING) {
        return;
    }

    uint32_t error = huart->ErrorCode;
    uint32_t current_time = osKernelGetTickCount();

    if (current_time - instance->last_error_timestamp > RC_UART_ERROR_VALIDITY_MS) {
        // Reset error count if last error was too long ago
        instance->error_count = 0;
    }
    instance->last_error_timestamp = current_time;
    instance->uart_error_code = error;
    instance->error_count++;

    // If more than RC_MAX_UART_ERROR_COUNT errors have occurred, within RC_UART_ERROR_VALIDITY_MS of each other, raise the error flag
    if (instance->error_count >= RC_MAX_UART_ERROR_COUNT) {
        LogWarning("RC Receiver: Too many UART errors, raising error flag");
        osThreadFlagsSet(instance->receiver_thread, RC_ERROR_THREAD_FLAG);
        instance->error_count = 0;  // Reset error count after raising the flag
    }

    switch (error) {
        case HAL_UART_ERROR_NONE:
            // No error
            break;
        case HAL_UART_ERROR_PE:
            LogVerbose("RC Receiver: UART Parity Error occurred");
            break;
        case HAL_UART_ERROR_NE:
            LogVerbose("RC Receiver: UART Noise Error occurred");
            break;
        case HAL_UART_ERROR_FE:
            LogVerbose("RC Receiver: UART Framing Error occurred");
            break;
        case HAL_UART_ERROR_ORE:
            LogVerbose("RC Receiver: UART Overrun Error occurred");
            break;
        case HAL_UART_ERROR_DMA:
            LogVerbose("RC Receiver: UART DMA Error occurred");
            break;
        default:
            LogDebug("RC Receiver: UART Unknown Error occurred, code: 0x%08lX", error);
            break;
    }

    // Reenable DMA reception after an error
    rx_callback_instance->enableUARTInterrupts();
}