/**
 * @file vesc_protocol.hpp
 *
 * @brief CAuDri - VESC 6 CAN Protocol Definitions
 *
 * Based on the official CAN format specifications from Vedder:
 * https://vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf
 * and the STM32F4 UART driver implementation:
 * https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery
 *
 */
#pragma once

#include <cstdint>

#define VESC_LOG_PARSE_INFO

#ifdef VESC_LOG_PARSE_INFO
    #define LogParse(...) LogInfo(__VA_ARGS__)
#else
    #define LogParse(...)
#endif

namespace vesc {

/**
 * @brief VESC CAN packet IDs
 *
 * These IDs are used to identify the type of data being sent or requested.
 */
enum PacketID : uint8_t {
    // Commands
    SET_DUTY = 0x00,                   // Set motor duty cycle
    SET_CURRENT = 0x01,                // Set motor current
    SET_CURRENT_BRAKE = 0x02,          // Set brake current
    SET_RPM = 0x03,                    // Set motor RPM
    SET_POS = 0x04,                    // Set motor position
    FILL_RX_BUFFER = 0x05,             // Fill RX buffer with data
    FILL_RX_BUFFER_LONG = 0x06,        // Fill RX buffer with long data
    PROCESS_RX_BUFFER = 0x07,          // Process RX buffer
    PROCESS_SHORT_BUFFER = 0x08,       // Process short buffer
    GET_STATUS = 0x09,                 // Request status packet
    SET_CURRENT_REL = 0x0A,            // Set relative current
    SET_CURRENT_BRAKE_REL = 0x0B,      // Set relative brake current
    SET_CURRENT_HANDBRAKE = 0x0C,      // Set handbrake current
    SET_CURRENT_HANDBRAKE_REL = 0x0D,  // Set relative
    STATUS_2 = 0x0E,                   // Request status packet 2
    STATUS_3 = 0x0F,                   // Request status packet 3
    STATUS_4 = 0x10,                   // Request status packet 4
    PING = 0x11,                       // Ping
    PONG = 0x12,                       // Pong
    DETECT_APPLY_ALL_FOC = 0x13,       // Detect and apply all FOC settings
    DETECT_APPLY_ALL_FOC_RES = 0x14,   // Result of detect and apply all FOC settings
    CONF_CURRENT_LIMITS = 0x15,        // Configure current limits
    CONF_STORE_CURRENT_LIMITS = 0x16, // Store current limits to flash
    CONF_CURRENT_LIMITS_IN = 0x17,     // Configure current limits (input)
    CONF_STORE_CURRENT_LIMITS_IN = 0x18, // Store current limits to flash
    CONF_FOC_ERPMS = 0x19,         // Configure FOC ERPM limits
    CONF_STORE_FOC_ERPMS = 0x1A,   // Store FOC ERPM limits to flash
    STATUS_5 = 0x1B,            // Request status packet 5
};

/**
 * @brief VESC Status Packet Structures
 *
 * These structures represent the data contained in various status packets
 * sent by the VESC over CAN.
 * 
 * Each VESC can be configured to publish different status messages periodically.
 * The user can choose which status messages to enable and at what frequency.
 */
struct Status1 {
    float rpm;        // Motor RPM
    float current;    // Motor current (A)
    float duty;       // Duty cycle (0.0-1.0)
};

struct Status2 {
    float amp_hours;         // Consumed amp hours (Ah)
    float amp_hours_charged; // Charged amp hours (Ah)
};

struct Status3 {
    float watt_hours;          // Consumed watt hours (Wh)
    float watt_hours_charged;  // Charged watt hours (Wh)
};

struct Status4 {
    float temp_fet;    // FET temperature (°C)
    float temp_motor;  // Motor temperature (°C)
    float current_in;  // Input current (A)
    float pid_pos_now; // Current PID position
};

struct Status5 {
    int32_t tacho_value; // Tachometer value
    float v_in;         // Input voltage (V)
};

struct Status6 {
    float adc_1; // ADC value 1
    float adc_2; // ADC value 2
    float adc_3; // ADC value 3
    float ppm;   // PPM value
};  

/**
 * @brief Combined VESC Status Structure
 *
 * This struct aggregates all individual status packets into one.
 * The driver can populate this struct by requesting each status packet.
 */
struct Status {
    Status1 status1;
    Status2 status2;
    Status3 status3;
    Status4 status4;
    Status5 status5;
    Status6 status6;
};



}  // namespace vesc