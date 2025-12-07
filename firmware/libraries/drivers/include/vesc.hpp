/**
 * @file vesc.hpp
 *
 * @brief CAuDri - VESC Driver
 */
#pragma once


#include "callback_wrapper.hpp"
#include "cmsis_os.h"
#include "driver.hpp"
#include "stm32f4xx_hal.h"
#include "vesc_protocol.hpp"

// Stack size for the VESC driver thread in bytes
constexpr uint32_t VESC_THREAD_STACK_SIZE = 1024;

// Hard limits for VESC config parameters
// Limits set in the configuration struct can not exceed these values
static constexpr int32_t VESC_MAX_RPM = 20000;          // Maximum allowable RPM
static constexpr float VESC_MAX_DUTY_CYCLE = 1.0f;      // Maximum allowable duty cycle (100%)
static constexpr float VESC_MAX_CURRENT_LIMIT = 50.0f;  // Maximum allowable motor current in Amperes

static constexpr uint32_t VESC_DEFAULT_STATUS_RATE_HZ = 50;  // Default expected rate of status updates from the VESC in Hz

class VESC : public Driver {
   public:
    /**
     * @brief Configuration struct for the VESC driver
     *
     * @param hcan Pointer to the initialized CAN handle
     * @param vesc_id The VESC ID on the CAN bus (1-255)
     * @param max_rpm (Optional) Maximum allowable RPM for the motor
     * @param max_duty_cycle (Optional) Maximum allowable duty cycle (0.0 to 1.0)
     * @param current_limit (Optional) Maximum allowable motor current in Amperes
     */
    struct Config {
        CAN_HandleTypeDef* hcan = nullptr;
        uint8_t vesc_id = 0;

        int32_t max_rpm = VESC_MAX_RPM;
        float max_duty_cycle = VESC_MAX_DUTY_CYCLE;
        float current_limit = VESC_MAX_CURRENT_LIMIT;

        uint32_t expected_status_rate_hz = 50;  // Expected rate of status updates from the VESC in Hz
    };

    VESC(const char* name);
    VESC(const char* name, const Config& config);
    ~VESC();

    bool init(const Config& config);
    bool start();

    const Config& getConfig() const { return config; }

    bool setDutyCycle(float duty_cycle);
    bool setRPM(int32_t rpm);
    bool setCurrent(float current_a);
    bool setPosition(float position_rad);

    bool getDutyCycle(float& duty_cycle) const;
    bool getRPM(float& rpm) const;
    bool getCurrent(float& current) const;
    bool getInputCurrent(float& current) const;
    bool getPosition(float& position) const;

    float getInputVoltage() const;
    float getMotorTemp() const;
    float getFETTemp() const;
    float getConsumedAmpHours() const;

    // TODO: Temp
    vesc::Status* getRawStatus() { return &vesc_status; }

   private:
    Config config;

    osThreadId_t vesc_thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[VESC_THREAD_STACK_SIZE / 4]{};

    uint8_t can_filter_bank = 0xFF;
    CallbackWrapper<void(const CAN_RxHeaderTypeDef*, const uint8_t*)> receive_callback;

    vesc::Status vesc_status{};
    std::array<uint32_t, 6> vesc_status_timestamps{};
    uint32_t expected_status_interval_ms = 0;

    static std::array<VESC*, 28> filter_bank_map;  // Map of CAN filter banks to VESC instances

    bool setConfig(const Config& config);
    bool initCAN();
    bool setCANMessageFilter();

    bool sendMessage(uint8_t vesc_id, vesc::PacketID packet_id, const uint8_t* data, uint8_t data_length);
    bool parseReceivedMessage(const CAN_RxHeaderTypeDef* header, const uint8_t* data);
    bool handleStatusPacket(const uint8_t* data, uint16_t length, uint8_t status_id);
    bool handlePingPacket(const uint8_t* data, uint16_t length);

    void vescThread(void* argument);

    static void receiveCallback(CAN_RxHeaderTypeDef header, uint8_t* data);
};