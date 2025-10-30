/**
 * @file vesc.cpp
 *
 * @brief CAuDri - VESC Driver Implementation
 */
#include "vesc.hpp"

#include "logger.h"
#include "thread_safe_can.h"

#define LOG_VERBOSE

#ifdef LOG_VERBOSE
    #define LogVerbose(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerbose(...)
#endif

VESC::VESC(const char* name) : Driver(name) {}

VESC::VESC(const char* name, const Config& config) : Driver(name), config(config) { init(config); }

VESC::~VESC() {}

bool VESC::init(const Config& config) {
    if (state != State::UNINITIALIZED) {
        LogError("%s: Driver already initialized", getName());
        return false;
    }

    if (config.hcan == nullptr || config.hcan->Instance == nullptr) {
        LogError("%s: Invalid CAN handle", getName());
        setState(State::ERROR);
        return false;
    }

    if (config.vesc_id == 0 || config.vesc_id > 0xFF) {
        LogError("%s: Invalid VESC ID %u, must be in range 1-255", getName(), config.vesc_id);
        setState(State::ERROR);
        return false;
    }

    this->config = config;

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

bool VESC::setRPM(int32_t rpm) {
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(rpm & 0xFF);
    data[1] = static_cast<uint8_t>((rpm >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((rpm >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((rpm >> 24) & 0xFF);

    return sendMessage(config.vesc_id, vesc::PacketID::SET_RPM, data, sizeof(data));
}

bool VESC::setDutyCycle(float duty_cycle) {
    int32_t duty_scaled = static_cast<int32_t>(duty_cycle * 100000.0f);
    uint8_t data[4];
    data[0] = static_cast<uint8_t>(duty_scaled & 0xFF);
    data[1] = static_cast<uint8_t>((duty_scaled >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((duty_scaled >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((duty_scaled >> 24) & 0xFF);

    return sendMessage(config.vesc_id, vesc::PacketID::SET_DUTY, data, sizeof(data));
}

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

bool VESC::setCANMessageFilter() {
    // The VESC exclusively uses 29-bit extended message IDs (CAN extended frame format).
    // The VESC ID and message type are encoded in the ID as follows:
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

    uint32_t flags = osThreadFlagsSet(vesc_thread_id, VESC_START_THREAD_FLAG);
    if (flags != VESC_START_THREAD_FLAG) {
        LogError("%s: Failed to start VESC driver thread, flags: 0x%08lX", getName(), flags);
        return false;
    }
    return true;
}

bool VESC::sendMessage(uint8_t vesc_id, vesc::PacketID packet_id, const uint8_t* data, uint8_t data_length) {
    if (getState() != State::RUNNING) {
        return false;
    }
    if (data == nullptr) {
        LogError("%s: Data pointer is null", getName());
        return false;
    }
    if (data_length == 0) {
        LogError("%s: Data length is 0", getName());
        return false;
    }
    if (data_length > 8) {
        LogError("%s: Data length %u exceeds maximum CAN payload size", getName(), data_length);
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
        LogError("%s: Failed to send CAN message (ID: 0x%08lX), status: %d",
                 getName(),
                 extended_id,
                 static_cast<int>(status));
        return false;
    }

    LogVerbose("%s: Sent CAN message (ID: 0x%08lX, DLC: %u)", getName(), extended_id, data_length);
    return true;
}

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

    LogVerbose("%s: Received CAN message (VESC ID: %u, Packet ID: 0x%02X, DLC: %u)",
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
    switch (packet_id) {
        // TODO: Implement handling for specific packet IDs
        default:
            LogWarning("%s: Unhandled packet ID 0x%02X", getName(), static_cast<uint8_t>(packet_id));
            break;
    }

    return true;
}

void VESC::vescThread(void* argument) {
    LogDebug("%s: VESC driver thread started", getName());

    // Wait for the start signal
    uint32_t flags = osThreadFlagsWait(VESC_START_THREAD_FLAG, osFlagsWaitAny, osWaitForever);
    if ((flags & VESC_START_THREAD_FLAG) == 0) {
        LogError("%s: Unexpected flags on start: 0x%08lX", getName(), flags);
        setState(State::ERROR);
        return;
    }

    setState(State::RUNNING);
    LogInfo("%s: Driver started", getName());

    while (1) {
        // Main driver loop
        osDelay(1000);
    }
}

void VESC::receiveCallback(const CAN_RxHeaderTypeDef* header, const uint8_t* data) {
    LogVerbose("%s: CAN receive callback triggered", getName());

    if (!parseReceivedMessage(header, data)) {
        LogError("%s: Failed to parse received CAN message", getName());
    }
}