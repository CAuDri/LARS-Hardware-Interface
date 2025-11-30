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

// Hard limits for VESC config parameters
// Limits set in the configuration struct can not exceed these values
constexpr int32_t VESC_MAX_RPM = 20000;          // Maximum allowable RPM
constexpr float VESC_MAX_DUTY_CYCLE = 1.0f;      // Maximum allowable duty cycle (100%)
constexpr float VESC_MAX_CURRENT_LIMIT = 50.0f;  // Maximum allowable motor current in Amperes

constexpr uint32_t VESC_INITIAL_CONNECT_TIMEOUT_MS = 5000;  // Timeout for receiving the first message from the VESC
constexpr uint32_t VESC_CONNECTION_LOST_TIMEOUT_MS = 1000;  // Timeout for considering the connection lost
constexpr uint32_t VESC_RECONNECT_TIMEOUT_MS = 5000;        // Time before considering the reconnect attempt failed

// Thread flags for notifying the VESC driver thread
constexpr uint32_t VESC_START_FLAG = 0x01;
constexpr uint32_t VESC_STATUS_UPDATE_FLAG = 0x02;

// Stack size for the VESC driver thread in bytes
constexpr uint32_t VESC_THREAD_STACK_SIZE = 1024;

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
    };

    VESC(const char* name);
    VESC(const char* name, const Config& config);
    ~VESC();

    bool init(const Config& config);
    bool start();

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

    bool setConfig(const Config& config);
    bool initCAN();
    bool setCANMessageFilter();

    bool sendMessage(uint8_t vesc_id, vesc::PacketID packet_id, const uint8_t* data, uint8_t data_length);
    bool parseReceivedMessage(const CAN_RxHeaderTypeDef* header, const uint8_t* data);
    bool handleStatusPacket(const uint8_t* data, uint16_t length, uint8_t status_id);
    bool handlePingPacket(const uint8_t* data, uint16_t length);

    void vescThread(void* argument);

    void receiveCallback(const CAN_RxHeaderTypeDef* header, const uint8_t* data);
};