/**
 * @file vesc.cpp
 *
 * @brief CAuDri - VESC Driver Implementation
 */
#include "vesc.hpp"

#include "logger.h"
#include "thread_safe_can.h"

// #define LOG_VERBOSE

#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

/**
 * @brief Construct a new VESC driver object for static initialization
 *
 * @param name Name of the driver instance
 */
VESC::VESC(const char* name) : Driver(name) {}

/**
 * @brief Construct a new VESC driver object with configuration
 *
 * @param name Name of the driver instance
 * @param config Configuration parameters for the VESC driver
 */
VESC::VESC(const char* name, const Config& config) : Driver(name), config(config) { init(config); }

VESC::~VESC() {}

/**
 * @brief Initialize the VESC driver with the provided configuration
 *
 * @param config Configuration parameters for the VESC driver
 * @return true if initialization was successful
 */
bool VESC::init(const Config& config) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Driver already initialized", getName());
        return false;
    }
    if (!setConfig(config)) {
        setState(State::ERROR);
        return false;
    }
    if (!initCAN()) {
        LogError("%s: Failed to initialize CAN interface", getName());
        setState(State::ERROR);
        return false;
    }

    // Configure and create the driver thread
    thread_attributes.name = getName();
    thread_attributes.priority = osPriorityNormal;
    thread_attributes.stack_mem = &thread_stack;
    thread_attributes.stack_size = sizeof(thread_stack);
    thread_attributes.cb_mem = &thread_control_block;
    thread_attributes.cb_size = sizeof(thread_control_block);

    vesc_thread_id = osThreadNew(
        // Helper function for using a non-static method as the thread entry point
        // The 'this' pointer is passed as the user argument to the lambda
        [](void* arg) -> void {
            auto* obj = static_cast<VESC*>(arg);
            obj->vescThread(arg);
        },
        this,
        &thread_attributes);

    if (vesc_thread_id == nullptr) {
        LogError("%s: Failed to create thread for the VESC driver", getName());
        setState(State::ERROR);
        return false;
    }

    LogInfo("%s: Driver initialized", getName());
    setState(State::INITIALIZED);
    return true;
}

/**
 * @brief Validate and set the VESC driver configuration
 *
 * @param config Configuration parameters for the VESC driver
 * @return true if the configuration was valid and set
 */
bool VESC::setConfig(const Config& config) {
    if (config.hcan == nullptr || config.hcan->Instance == nullptr) {
        LogError("%s: Invalid CAN handle configuration", getName());
        return false;
    }

    if (config.vesc_id == 0 || config.vesc_id > 0xFF) {
        LogError("%s: Invalid VESC ID (%u) configuration, must be in range 1-255", getName(), config.vesc_id);
        return false;
    }

    if (config.max_rpm < 0 || config.max_rpm > VESC_MAX_RPM) {
        LogError("%s: Configured max RPM (%lu) exceeds allowable limit (%lu)", getName(), config.max_rpm, VESC_MAX_RPM);
        return false;
    }

    if (config.max_duty_cycle < 0.0f || config.max_duty_cycle > VESC_MAX_DUTY_CYCLE) {
        LogError("%s: Configured max duty cycle (%.2f) exceeds allowable limit (%.2f)",
                 getName(),
                 config.max_duty_cycle,
                 VESC_MAX_DUTY_CYCLE);
        return false;
    }

    if (config.current_limit < 0.0f || config.current_limit > VESC_MAX_CURRENT_LIMIT) {
        LogError("%s: Configured current limit (%.2f A) exceeds allowable limit (%.2f A)",
                 getName(),
                 config.current_limit,
                 VESC_MAX_CURRENT_LIMIT);
        return false;
    }

    this->config = config;
    return true;
}

/**
 * @brief Initialize the CAN interface for VESC communication
 *
 * It will set up a CAN message filter for all messages coming from the configured VESC ID
 * and register a receive callback to handle incoming messages.
 *
 * @return true if CAN interface was initialized successfully
 */
bool VESC::initCAN() {
    // Set up CAN message filter for VESC messages
    if (!setCANMessageFilter()) {
        LogError("%s: Failed to set CAN message filter", getName());
        return false;
    }

    if (can_filter_bank > 27) {
        LogError("%s: Invalid CAN filter bank %u after allocation", getName(), can_filter_bank);
        return false;
    }

    // Register the receive callback for the allocated filter bank
    receive_callback.from<VESC, &VESC::receiveCallback>(this);
    CAN_RxCallback_t receive_callback_handle = (CAN_RxCallback_t)(receive_callback.c_callback());

    if (CAN_RegisterRxCallback(config.hcan, can_filter_bank, receive_callback_handle) != HAL_OK) {
        LogError("%s: Failed to register RX callback for CAN filter bank %u", getName(), can_filter_bank);
        return false;
    }

    return true;
}

/**
 * @brief Configure the CAN message filter for receiving VESC messages
 *
 * @return true if the filter was configured successfully
 */
bool VESC::setCANMessageFilter() {
    // The VESC exclusively uses 29-bit extended message IDs (CAN extended frame format).
    // The VESC ID and message type are encoded in the ID:
    // Bits [7:0]   : VESC ID
    // Bits [15:8]  : Message Type
    // The rest of the bits are reserved.
    //
    // The bxCAN peripheral supports filtering based on a mask and ID.
    // It will match a 32-bit word from the message (ID[31:3] + RTR[2] + IDE[1]) against the filter.
    uint32_t filter_id;
    uint32_t filter_mask;

    // We want to accept messages with our VESC ID, regardless of message type
    // The first 8 bits are the VESC ID, which need to match exactly
    // They are located at bits [10:3] in the 32-bit words used for filtering
    filter_id = ((static_cast<uint32_t>(config.vesc_id) & 0xFF) << 3);
    filter_mask = (0xFF << 3);  // Masked pins will be compared, unmasked pins are wildcards

    // We only want to accept extended IDs (IDE = 1)
    filter_id |= (1 << 1);
    filter_mask |= (1 << 1);

    CAN_FilterTypeDef can_filter{};

    // We will use the helper function from the HAL wrapper to allocate a filter bank
    can_filter.FilterBank = 0;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterActivation = ENABLE;

    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

    can_filter.FilterIdHigh = static_cast<uint16_t>(filter_id >> 16);    // 16 MSBs
    can_filter.FilterIdLow = static_cast<uint16_t>(filter_id & 0xFFFF);  // 16 LSBs
    can_filter.FilterMaskIdHigh = static_cast<uint16_t>(filter_mask >> 16);
    can_filter.FilterMaskIdLow = static_cast<uint16_t>(filter_mask & 0xFFFF);

    HAL_StatusTypeDef status = CAN_ConfigAndAllocateFilter(config.hcan, &can_filter);
    if (status != HAL_OK) {
        LogError("%s: CAN filter configuration failed with status %d", getName(), static_cast<int>(status));
        return false;
    }
    LogDebug("%s: CAN filter configured (ID: 0x%08lX, Mask: 0x%08lX)", getName(), filter_id, filter_mask);

    this->can_filter_bank = can_filter.FilterBank;

    return true;
};

bool VESC::start() {
    if (state != State::INITIALIZED) {
        LogWarning("%s: Cannot start, driver not initialized", getName());
        return false;
    }

    uint32_t flags = osThreadFlagsSet(vesc_thread_id, VESC_START_FLAG);
    if (flags != VESC_START_FLAG) {
        LogError("%s: Failed to start VESC driver thread, flags: 0x%08lX", getName(), flags);
        return false;
    }
    return true;
}

/**
 * @brief Send a CAN message to the VESC
 *
 * @param vesc_id The VESC ID to send the message to
 * @param packet_id The VESC packet ID (message type)
 * @param data Pointer to the data payload
 * @param data_length Length of the data payload in bytes (max 8)
 * @return true if the message was sent successfully
 */
bool VESC::sendMessage(uint8_t vesc_id, vesc::PacketID packet_id, const uint8_t* data, uint8_t data_length) {
    if (state != State::RUNNING || connection_state != ConnectionState::CONNECTED) {
        return false;
    }
    if (data == nullptr) {
        LogWarning("%s: Data pointer is null", getName());
        return false;
    }
    if (data_length == 0) {
        LogWarning("%s: Data length is 0", getName());
        return false;
    }
    if (data_length > 8) {
        LogWarning("%s: Data length %u exceeds maximum CAN payload size", getName(), data_length);
        return false;
    }

    // The message type (packet_id) and VESC ID are encoded in the extended CAN ID:
    // Bits [7:0]   : VESC ID
    // Bits [15:8]  : Message Type
    uint32_t extended_id = (static_cast<uint32_t>(packet_id) << 8) | static_cast<uint32_t>(vesc_id);

    CAN_TxHeaderTypeDef tx_header{};
    tx_header.StdId = 0;  // Not used for extended IDs
    tx_header.ExtId = extended_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = data_length;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;
    HAL_StatusTypeDef status =
        CAN_AddTxMessage(config.hcan, &tx_header, const_cast<uint8_t*>(data), &tx_mailbox);
    if (status != HAL_OK) {
        LogWarning("%s: Failed to send CAN message (ID: 0x%08lX), status: %d",
                 getName(),
                 extended_id,
                 static_cast<int>(status));
        return false;
    }

    LogVerbose("%s: Sent CAN message (ID: 0x%08lX, DLC: %u)", getName(), extended_id, data_length);
    return true;
}

/**
 * @brief Parse a received CAN message from the VESC
 *
 * @param header Pointer to the received CAN Rx header
 * @param data Pointer to the received CAN data payload
 * @return true if the message was parsed successfully
 */
bool VESC::parseReceivedMessage(const CAN_RxHeaderTypeDef* header, const uint8_t* data) {
    if (getState() != State::RUNNING) {
        return false;
    }
    if (header == nullptr || data == nullptr) {
        LogError("%s: Received null pointer when parsing CAN message", getName());
        return false;
    }

    // Extract VESC ID and packet ID from the extended CAN ID
    uint8_t vesc_id = static_cast<uint8_t>(header->ExtId & 0xFF);
    vesc::PacketID packet_id = static_cast<vesc::PacketID>((header->ExtId >> 8) & 0xFF);

    LogVerbose("%s: Received CAN message (VESC ID: %u, Packet ID: 0x%02X, DLC: %lu)",
               getName(),
               vesc_id,
               static_cast<uint8_t>(packet_id),
               header->DLC);

    if (vesc_id != config.vesc_id) {
        LogWarning("%s: Received message from VESC ID %u, but configured for ID %u",
                   getName(),
                   vesc_id,
                   config.vesc_id);
        return false;
    }

    // Handle the received message based on its packet ID
    bool success = false;
    switch (packet_id) {
        case vesc::PacketID::STATUS_1:
            success = handleStatusPacket(data, header->DLC, 1);
            break;
        case vesc::PacketID::STATUS_2:
            success = handleStatusPacket(data, header->DLC, 2);
            break;
        case vesc::PacketID::STATUS_3:
            success = handleStatusPacket(data, header->DLC, 3);
            break;
        case vesc::PacketID::STATUS_4:
            success = handleStatusPacket(data, header->DLC, 4);
            break;
        case vesc::PacketID::STATUS_5:
            success = handleStatusPacket(data, header->DLC, 5);
            break;
        // TODO: Add additional status packets
        // case vesc::PacketID::STATUS_6:
        //     success = handleStatusPacket(data, header->DLC, 6);
        //     break;
        case vesc::PacketID::PING:
            success = handlePingPacket(data, header->DLC);
            break;
        default:
            LogWarning("%s: Unhandled packet ID 0x%02X", getName(), static_cast<uint8_t>(packet_id));
            break;
    }

    return success;
}

bool VESC::handleStatusPacket(const uint8_t* data, uint16_t length, uint8_t status_id) {
    if (data == nullptr || length == 0) {
        LogWarning("%s: Invalid data pointer or length for status packet %u", getName(), status_id);
        return false;
    }
    LogVerbose("%s: Handling status packet %u", getName(), status_id);

    switch (status_id) {
        case 1: {
            if (length < 8) {
                LogDebug("%s: Status packet 1 length %u is less than expected 8 bytes", getName(), length);
                return false;
            }
            // RPM: int32_t (4 bytes)
            // Current: int16_t (2 bytes) scaled by 10
            // Duty Cycle: uint16_t (2 bytes) scaled by 1000
            int32_t rpm = (static_cast<int32_t>(data[0]) << 24) | (static_cast<int32_t>(data[1]) << 16) |
                          (static_cast<int32_t>(data[2]) << 8) | static_cast<int32_t>(data[3]);
            int16_t current = (static_cast<int16_t>(data[4]) << 8) | static_cast<int16_t>(data[5]);
            int16_t duty_cycle = (static_cast<int16_t>(data[6]) << 8) | static_cast<int16_t>(data[7]);

            vesc_status.status1.rpm = rpm;
            vesc_status.status1.current = static_cast<float>(current) / 10.0f;
            vesc_status.status1.duty = static_cast<float>(duty_cycle) / 1000.0f;
            vesc_status_timestamps[0] = osKernelGetTickCount();
            break;
        }
        case 2: {
            if (length < 8) {
                LogDebug("%s: Status packet 2 length %u is less than expected 8 bytes", getName(), length);
                return false;
            }
            // Amp-hours consumed: uint32_t (4 bytes) scaled by 10000
            // Amp-hours charged: uint32_t (4 bytes) scaled by 10000
            uint32_t amp_hours_consumed =
                (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
                (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
            uint32_t amp_hours_charged =
                (static_cast<uint32_t>(data[4]) << 24) | (static_cast<uint32_t>(data[5]) << 16) |
                (static_cast<uint32_t>(data[6]) << 8) | static_cast<uint32_t>(data[7]);

            vesc_status.status2.amp_hours = static_cast<float>(amp_hours_consumed) / 10000.0f;
            vesc_status.status2.amp_hours_charged = static_cast<float>(amp_hours_charged) / 10000.0f;
            vesc_status_timestamps[1] = osKernelGetTickCount();
            break;
        }
        default:
            LogWarning("%s: Unhandled status packet ID %u", getName(), status_id);
            return false;
    }

    uint32_t flags = osThreadFlagsSet(vesc_thread_id, VESC_STATUS_UPDATE_FLAG);
    if ((flags & VESC_STATUS_UPDATE_FLAG) == 0) {
        LogError("%s: Failed to notify VESC thread of status update, flags: 0x%08lX", getName(), flags);
        return false;
    }

    return true;
}

bool VESC::handlePingPacket(const uint8_t* data, uint16_t length) {
    LogVerbose("%s: Ping received from VESC", getName());
    // TODO: Find out how to respond to pings if necessary
    return true;
}

/**
 * @brief Set the motor RPM on the VESC
 *
 * @param rpm Desired motor RPM
 * @return true if the command was sent successfully
 */
bool VESC::setRPM(int32_t rpm) {
    if (state != State::RUNNING) {
        return false;
    }

    // Enforce maximum RPM limit from configuration
    if (rpm > config.max_rpm) {
        rpm = config.max_rpm;
    } else if (rpm < -config.max_rpm) {
        rpm = -config.max_rpm;
    }

    // VESC expects RPM as a 32-bit signed integer in big-endian format
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(rpm >> 24 & 0xFF);
    data[1] = static_cast<uint8_t>(rpm >> 16 & 0xFF);
    data[2] = static_cast<uint8_t>(rpm >> 8 & 0xFF);
    data[3] = static_cast<uint8_t>(rpm & 0xFF);

    LogVerbose("%s: Setting RPM to %ld", getName(), rpm);
    return sendMessage(config.vesc_id, vesc::PacketID::SET_RPM, data, sizeof(data));
}

/**
 * @brief Set the motor duty cycle on the VESC
 *
 * @param duty_cycle Desired duty cycle (0.0 to 1.0)
 * @return true if the command was sent successfully
 */
bool VESC::setDutyCycle(float duty_cycle) {
    if (state != State::RUNNING) {
        return false;
    }

    // Enforce maximum duty cycle limit from configuration
    if (duty_cycle > config.max_duty_cycle) {
        duty_cycle = config.max_duty_cycle;
    } else if (duty_cycle < -config.max_duty_cycle) {
        duty_cycle = -config.max_duty_cycle;
    }

    // VESC expects duty cycle as a signed integer scaled by 100000 in big-endian format
    int32_t duty_scaled = static_cast<int32_t>(duty_cycle * 100000.0f);
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(duty_scaled >> 24 & 0xFF);
    data[1] = static_cast<uint8_t>(duty_scaled >> 16 & 0xFF);
    data[2] = static_cast<uint8_t>(duty_scaled >> 8 & 0xFF);
    data[3] = static_cast<uint8_t>(duty_scaled & 0xFF);

    LogVerbose("%s: Setting duty cycle to %.2f", getName(), duty_cycle);
    return sendMessage(config.vesc_id, vesc::PacketID::SET_DUTY, data, sizeof(data));
}

/**
 * @brief Set the motor current on the VESC
 *
 * @param current Desired motor current in Amperes
 * @return true if the command was sent successfully
 */
bool VESC::setCurrent(float current) {
    if (state != State::RUNNING) {
        return false;
    }

    // Enforce current limit from configuration
    if (current > config.current_limit) {
        current = config.current_limit;
    } else if (current < -config.current_limit) {
        current = -config.current_limit;
    }

    // VESC expects current as a signed integer scaled by 1000 in big-endian format
    int32_t current_scaled = static_cast<int32_t>(current * 1000.0f);
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(current_scaled >> 24 & 0xFF);
    data[1] = static_cast<uint8_t>(current_scaled >> 16 & 0xFF);
    data[2] = static_cast<uint8_t>(current_scaled >> 8 & 0xFF);
    data[3] = static_cast<uint8_t>(current_scaled & 0xFF);

    LogVerbose("%s: Setting current to %.2f A", getName(), current);
    return sendMessage(config.vesc_id, vesc::PacketID::SET_CURRENT, data, sizeof(data));
}

/**
 * @brief Thread function for managing VESC communication and connection state
 * 
 * @param argument Pointer to the thread argument (unused)
 */
void VESC::vescThread(void* argument) {
    LogDebug("%s: VESC driver thread started", getName());

    // Wait for the start signal from the main application
    uint32_t flags = osThreadFlagsWait(VESC_START_FLAG, osFlagsWaitAny, osWaitForever);
    if (!(flags & VESC_START_FLAG) || (flags & osFlagsError)) {
        LogError("%s: Error waiting for start flag: 0x%08lX", getName(), flags);
        setState(State::ERROR);
        osDelay(osWaitForever);
    }


    setState(State::RUNNING);
    setConnectionState(ConnectionState::CONNECTED);
    osDelay(osWaitForever);



    setState(State::RUNNING);
    setConnectionState(ConnectionState::CONNECTING);
    LogInfo("%s: Driver started", getName());

    // We should now be able to receive CAN messages and will wait for the first status update
    flags = osThreadFlagsWait(VESC_STATUS_UPDATE_FLAG, osFlagsWaitAny, VESC_INITIAL_CONNECT_TIMEOUT_MS);
    if (flags == osFlagsErrorTimeout) {
        LogError("%s: Connection to the VESC could not be established within %lu ms, shutting down",
                 getName(),
                 VESC_INITIAL_CONNECT_TIMEOUT_MS);
        setConnectionState(ConnectionState::DISCONNECTED);
        setState(State::ERROR);
        osDelay(osWaitForever);
    } else if (!(flags & VESC_STATUS_UPDATE_FLAG) || (flags & osFlagsError)) {
        LogError("%s: Error waiting for initial status update flag: 0x%08lX", getName(), flags);
        setState(State::ERROR);
        osDelay(osWaitForever);
    }

    // TODO: Ping or validate connection further

    setConnectionState(ConnectionState::CONNECTED);
    LogInfo("%s: Connection to the VESC established", getName());

    while (getState() == State::RUNNING) {
        switch (getConnectionState()) {
            case ConnectionState::CONNECTED:
                // Any time a status update is received, the receive callback will set the status update flag
                // If no updates are received within VESC_CONNECTION_LOST_TIMEOUT_MS, we try to reconnect
                flags = osThreadFlagsWait(VESC_STATUS_UPDATE_FLAG, osFlagsWaitAny, VESC_CONNECTION_LOST_TIMEOUT_MS);

                if (flags == osFlagsErrorTimeout) {
                    LogWarning("%s: Connection to the VESC lost, attempting to reconnect", getName());
                    setConnectionState(ConnectionState::CONNECTING);
                }
                break;

            case ConnectionState::CONNECTING:
                // Wait for the next status update to re-establish the connection
                flags = osThreadFlagsWait(VESC_STATUS_UPDATE_FLAG, osFlagsWaitAny, VESC_RECONNECT_TIMEOUT_MS);

                if (flags == osFlagsErrorTimeout) {
                    LogError("%s: Reconnection attempt to the VESC timed out after %lu ms", getName(), VESC_RECONNECT_TIMEOUT_MS);
                    setConnectionState(ConnectionState::DISCONNECTED);
                    setState(State::ERROR);
                } else if (flags & VESC_STATUS_UPDATE_FLAG) {
                    LogInfo("%s: Reconnected to the VESC", getName());
                    setConnectionState(ConnectionState::CONNECTED);
                } else {
                    LogError("%s: Error waiting for status update during reconnection: 0x%08lX", getName(), flags);
                    setState(State::ERROR);
                }
                break;

            default:
                LogError("%s: Invalid connection state in VESC driver thread", getName());
                setState(State::ERROR);
                break;
        }
    }

    LogDebug("%s: VESC driver thread exiting", getName());
    osDelay(osWaitForever);
}

/**
 * @brief Callback function for received CAN messages
 *
 * @param header Pointer to the received CAN Rx header
 * @param data Pointer to the received CAN data payload
 */
void VESC::receiveCallback(const CAN_RxHeaderTypeDef* header, const uint8_t* data) {
    LogVerbose("%s: CAN receive callback triggered", getName());

    if (getState() != State::RUNNING) {
        return;
    }
    if (getConnectionState() != ConnectionState::CONNECTED && getConnectionState() != ConnectionState::CONNECTING) {
        return;
    }
    if (!parseReceivedMessage(header, data)) {
        LogError("%s: Failed to parse received CAN message", getName());
        return;
    }
}