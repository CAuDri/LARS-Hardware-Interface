/**
 * @file crsf_protocol.hpp
 *
 * @brief CAuDri - CRSF Protocol Definitions
 *
 * Based on the official CRSF protocol documentation from Team BlackSheep:
 * https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
 *
 * This is only a subset of the full protocol, with all necessary features for LARS or other RC vehicles.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Uncomment to enable detailed parsing logs
// #define CRSF_LOG_PARSE_INFO

#ifdef CRSF_LOG_PARSE_INFO
    #define LogVerboseCRSF(...) LogDebug(__VA_ARGS__)
#else
    #define LogVerboseCRSF(...)
#endif

namespace crsf {

/**
 * @brief CRSF device addresses
 */
enum Address : uint8_t {
    BROADCAST = 0x00,          // Frame sent to all devices
    FLIGHT_CONTROLLER = 0x80,  // Frame sent to the "flight controller" (our MCU)
    TRANSMITTER = 0xEE,        // Frame sent to the transmitter (remote controller)
    RECEIVER = 0xEC            // Frame sent to the actual receiver module
};

/**
 * @brief CRSF frame type subset
 */
enum FrameType : uint8_t {
    LINK_STATISTICS = 0x14,     // Link statistics frame (RSSI, SNR, etc.) from the receiver
    RC_CHANNELS_PACKED = 0x16,  // RC channel data frame (16 channels, 11-bit resolution)
    HEARTBEAT = 0x0B,           // Heartbeat frame

    // Telemetry frame types (vehicle to transmitter)
    RPM = 0x0C,      // RPM frame
    TEMP = 0x0D,     // Temperature frame
    VOLTAGE = 0x0E,  // Voltage frame

    // Extended frame types (require extended header)
    PING = 0x28,         // Ping specific address or all devices depending on destination address
    DEVICE_INFO = 0x29,  // Device information (Name, Version, ID, etc.)
};

constexpr uint32_t BAUD_RATE = 420000;  // Standard baud rate for CRSF communication

constexpr uint8_t SYNC_BYTE = 0xC8;          // Default sync byte for all CRSF frames
constexpr size_t MAX_TOTAL_FRAME_SIZE = 64;  // Maximum size of a CRSF frame (including header and CRC)
constexpr size_t MAX_PAYLOAD_SIZE = 58;      // Maximum payload size for a CRSF frame
constexpr size_t MIN_HEADER_SIZE = 3;        // Size of the CRSF frame header (sync, length, type)

constexpr size_t MIN_FRAME_LENGTH = 2;  // Minimum size of a CRSF frame (excluding sync and length bytes)
constexpr size_t MAX_FRAME_LENGTH = 62;  // Maximum size of a CRSF frame (excluding sync and length bytes)

constexpr uint8_t EXTENDED_FRAME_MIN_TYPE = 0x27;  // Minimum frame type value for extended frames

constexpr size_t RC_CHANNELS_PAYLOAD_SIZE = 22;      // Size of the RC_CHANNELS_PACKED frame
constexpr size_t LINK_STATISTICS_PAYLOAD_SIZE = 10;  // Size of the LINK_STATISTICS frame

constexpr uint32_t CONNECTION_TIMEOUT_MS = 100;  // Time without valid RC data before considering the link lost

/**
 * @brief Result of parsing a CRSF frame
 *
 * @param valid Whether the frame was valid (CRC + length check)
 * @param length Total length of the frame, excluding sync and length bytes
 * @param frame_type Type of the frame (crsf::FrameType)
 * @param extended Whether the frame uses the extended header format
 * @param source_address Source address of the frame
 * @param destination_address Destination address of the frame
 * @param payload Payload data of the frame
 */
struct ParseResult {
    bool valid = false;
    size_t length = 0;
    uint8_t frame_type = 0;
    bool extended = false;
    uint8_t source_address = 0;
    uint8_t destination_address = 0;
    std::array<uint8_t, MAX_PAYLOAD_SIZE> payload{};
};

/**
 * @brief Link statistics from the receiver module (Signal strength, SNR, etc.)
 *
 * @param uplink_rssi_1 Uplink RSSI (receiver to transmitter) Antenna 1 (dBm)
 * @param uplink_rssi_2 Uplink RSSI (receiver to transmitter) Antenna 2 (dBm)
 * @param uplink_quality Uplink Link Quality/Packet Success Rate (%)
 * @param uplink_snr Uplink Signal-to-Noise Ratio (dB)
 * @param active_antenna Active Antenna (0 -> Antenna 1, 1 -> Antenna 2)
 * @param rf_mode RF Mode (0-5 -> 4, 50, 150, 250, 500, 1000 FPS)
 * @param uplink_tx_power Uplink TX Power (0-7 -> 0, 10, 25,100, 250, 500, 1000, 2000 mW)
 * @param downlink_rssi Downlink RSSI (transmitter to receiver)
 * @param downlink_quality Downlink Link Quality/Packet Success Rate (%)
 * @param downlink_snr Downlink Signal-to-Noise Ratio (dB)
 */
struct LinkStatistics {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_quality;
    int8_t uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;

    uint8_t downlink_rssi;
    uint8_t downlink_quality;
    int8_t downlink_snr;
};

using ChannelData = std::array<uint16_t, 16>;  // Type for storing 16 RC channel values (11-bit each)

// CRC-8 lookup table for CRSF frames (Polynomial 0xD5)
constexpr uint8_t CRC8_TABLE[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

// Calculate CRC-8 for a given data buffer
inline uint8_t calcCRC(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

/**
 * @brief Parse a CRSF frame from the given data buffer
 *
 * It is expected that the data buffer contains at least one complete frame.
 * The function will validate the frame using CRC and length checks and populate the ParseResult struct.
 *
 * @param data Pointer to the data buffer containing the CRSF frame
 * @param length Length of the data buffer/frame
 * @param result Reference to a ParseResult struct to be populated with the parsed frame data
 *
 * @return true if the frame was successfully parsed and is valid, false otherwise
 */
inline bool parseFrame(uint8_t* data, size_t length, ParseResult& result) {
    if (data == nullptr) {
        LogVerboseCRSF("CRSF Parser: Null data pointer");
        return false;
    }
    if (length < MIN_HEADER_SIZE) {
        LogVerboseCRSF("CRSF Parser: Frame too short, length: %u", length);
        return false;
    }

    // Sync byte should always be the first byte of the frame.
    // It can be either the standard SYNC_BYTE or the address of the flight controller
    // (0x80). Some receivers may use the BROADCAST address (0x00) as a sync byte.
    uint8_t sync_byte = data[0];
    if (sync_byte != SYNC_BYTE && sync_byte != (Address::BROADCAST) && sync_byte != (Address::FLIGHT_CONTROLLER)) {
        LogVerboseCRSF("CRSF Parser: Received invalid sync byte: 0x%02X", sync_byte);
        return false;
    }

    // Length of the frame excluding sync and length bytes read from the message
    uint8_t frame_length = data[1];
    if (frame_length < MIN_FRAME_LENGTH || frame_length > MAX_FRAME_LENGTH) {
        LogVerboseCRSF("CRSF Parser: Received invalid frame length: %d", frame_length);
        return false;
    }

    if (length < frame_length + 2u) {
        LogVerboseCRSF("CRSF Parser: Incomplete frame received, expected length %d, got %d", frame_length + 2, length);
        // TODO: Handle incomplete frame (buffering, etc.)
        return false;
    }

    // Check the received frame for any errors during transmission or an invalid CRC
    // CRC includes type and payload, but not sync and frame length
    uint8_t received_crc = data[frame_length + 1];
    uint8_t calc_crc = calcCRC(&data[2], frame_length - 1);
    if (received_crc != calc_crc) {
        LogVerboseCRSF("CRSF Parser: Invalid CRC, received 0x%02X, calculated 0x%02X", received_crc, calc_crc);
        return false;
    }

    uint8_t frame_type = data[2];

    // The frame can now be considered valid and we can parse additional fields
    result.valid = true;
    result.length = frame_length;
    result.frame_type = frame_type;

    if (frame_type >= EXTENDED_FRAME_MIN_TYPE) {
        result.extended = true;
        result.source_address = data[3];
        result.destination_address = data[4];
        std::copy(&data[5], &data[5] + (frame_length - 4), result.payload.begin());
    } else {
        result.extended = false;
        result.source_address = sync_byte;  // In non-extended frames, the sync byte is the source address
        std::copy(&data[3], &data[3] + (frame_length - 2), result.payload.begin());
    }

    result.valid = true;
    result.length = frame_length;
    result.frame_type = frame_type;
    if (result.extended) {
        result.source_address = data[3];
        result.destination_address = data[4];
    }

    // LogVerboseCRSF(
    //     "CRSF Parser: Parsed frame type 0x%02X, length %d, extended %d, src 0x%02X, dst "
    //     "0x%02X",
    //     result.frame_type,
    //     result.length,
    //     result.extended,
    //     result.source_address,
    //     result.destination_address);

    return true;
}

/**
 * @brief Unpack 16 channels of 11-bit RC data from a packed byte array
 *
 * The CRSF RC_CHANNELS_PACKED frame contains 16 channels of RC data, each represented as an 11-bit value, LSB first.
 * The data is packed into a byte array, and this function extracts the individual channel values.
 *
 * @param payload Pointer to the packed byte array (22 bytes)
 * @param channels Output array to store the unpacked channel values (16 elements)
 */
inline void parseChannelData(const uint8_t* payload, ChannelData& channels) {
    if (payload == nullptr) {
        LogError("CRSF Parser: Invalid payload pointer for ChannelData");
        return;
    }
    constexpr uint32_t MASK_11_BIT = 0x7FF;

    uint32_t bit_pos = 0;
    for (size_t i = 0; i < 16; ++i) {
        size_t byte_index = bit_pos / 8;
        size_t bit_offset = bit_pos % 8;

        // 11 bits may span up to 3 bytes, so we use a 3-byte sliding window and store it in a 32-bit integer
        // For the last channel this will read beyond the data array bounds, but should be safe in our use case
        uint32_t value =
            (payload[byte_index]) | (payload[byte_index + 1] << 8) | (payload[byte_index + 2] << 16);

        channels[i] = (value >> bit_offset) & MASK_11_BIT;
        bit_pos += 11;
    }
}

/**
 * @brief Unpack link statistics from a LINK_STATISTICS frame payload
 *
 * The LINK_STATISTICS frame contains various signal quality metrics from the receiver module.
 * This function extracts the relevant fields and populates a LinkStatistics struct.
 *
 * @param payload Pointer to the payload data of the LINK_STATISTICS frame (at least 10 bytes)
 * @param stats Output struct to store the unpacked link statistics
 */
inline void parseLinkStatistics(const uint8_t* payload, LinkStatistics& stats) {
    if (payload == nullptr) {
        LogError("CRSF Parser: Invalid payload pointer for LinkStatistics");
        return;
    }

    stats.uplink_rssi_1 = payload[0];
    stats.uplink_rssi_2 = payload[1];
    stats.uplink_quality = payload[2];
    stats.uplink_snr = static_cast<int8_t>(payload[3]);
    stats.active_antenna = payload[4];
    stats.rf_mode = payload[5];
    stats.uplink_tx_power = payload[6];

    stats.downlink_rssi = payload[7];
    stats.downlink_quality = payload[8];
    stats.downlink_snr = static_cast<int8_t>(payload[9]);
}

}  // namespace crsf