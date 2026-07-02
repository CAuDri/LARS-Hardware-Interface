#define LogError(...) ((void)0)

#include "crsf_protocol.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace {

constexpr uint8_t SENTINEL = 0xA5;

void testStandardFrame() {
    std::array<uint8_t, crsf::MAX_TOTAL_FRAME_SIZE> buffer{};
    constexpr std::array<uint8_t, 3> payload{0x12, 0x34, 0x56};
    size_t length = 123;

    const bool composed = crsf::composeFrame(buffer.data(), buffer.size(), length, crsf::HEARTBEAT,
                                             payload.data(), payload.size(), crsf::TRANSMITTER, crsf::RECEIVER);

    assert(composed);
    assert(length == payload.size() + 4U);
    assert(buffer[0] == crsf::SYNC_BYTE);
    assert(buffer[1] == payload.size() + 2U);
    assert(buffer[2] == crsf::HEARTBEAT);
    assert(std::equal(payload.begin(), payload.end(), buffer.begin() + 3));
    assert(buffer[length - 1] == crsf::calcCRC(&buffer[2], buffer[1] - 1U));
}

void testEmptyPayload() {
    std::array<uint8_t, 4> buffer{};
    size_t length = 123;

    const bool composed = crsf::composeFrame(buffer.data(), buffer.size(), length, crsf::HEARTBEAT, nullptr, 0,
                                             crsf::TRANSMITTER, crsf::RECEIVER);

    assert(composed);
    assert(length == buffer.size());
    assert(buffer[1] == crsf::MIN_FRAME_LENGTH);
}

void testNullPayloadWithNonzeroLength() {
    std::array<uint8_t, crsf::MAX_TOTAL_FRAME_SIZE> buffer{};
    buffer.fill(SENTINEL);
    const auto original = buffer;
    size_t length = 123;

    const bool composed = crsf::composeFrame(buffer.data(), buffer.size(), length, crsf::HEARTBEAT, nullptr, 1,
                                             crsf::TRANSMITTER, crsf::RECEIVER);

    assert(!composed);
    assert(length == 0);
    assert(buffer == original);
}

void testUndersizedBuffer() {
    std::array<uint8_t, 4> buffer{};
    buffer.fill(SENTINEL);
    const auto original = buffer;
    constexpr std::array<uint8_t, 1> payload{0x42};
    size_t length = 123;

    const bool composed = crsf::composeFrame(buffer.data(), buffer.size(), length, crsf::HEARTBEAT,
                                             payload.data(), payload.size(), crsf::TRANSMITTER, crsf::RECEIVER);

    assert(!composed);
    assert(length == 0);
    assert(buffer == original);
}

void testMaximumExtendedFrame() {
    std::array<uint8_t, crsf::MAX_TOTAL_FRAME_SIZE> buffer{};
    std::array<uint8_t, crsf::MAX_PAYLOAD_SIZE> payload{};
    size_t length = 0;

    const bool composed = crsf::composeFrame(buffer.data(), buffer.size(), length, crsf::PING, payload.data(),
                                             payload.size(), crsf::TRANSMITTER, crsf::RECEIVER);

    assert(composed);
    assert(length == crsf::MAX_TOTAL_FRAME_SIZE);
    assert(buffer[1] == crsf::MAX_FRAME_LENGTH);
    assert(buffer[2] == crsf::PING);
    assert(buffer[3] == crsf::TRANSMITTER);
    assert(buffer[4] == crsf::RECEIVER);
}

}  // namespace

int main() {
    testStandardFrame();
    testEmptyPayload();
    testNullPayloadWithNonzeroLength();
    testUndersizedBuffer();
    testMaximumExtendedFrame();
    return 0;
}
