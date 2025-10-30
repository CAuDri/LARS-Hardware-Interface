/**
 * @file vesc.hpp
 *
 * @brief CAuDri - VESC Driver
 */
#pragma once


#include "driver.hpp"
#include "callback_wrapper.hpp"
#include "vesc_protocol.hpp"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

// Stack size for the VESC driver thread in bytes
constexpr uint32_t VESC_THREAD_STACK_SIZE = 1024;

// Thread flags for notifying the VESC driver thread
constexpr uint32_t VESC_START_THREAD_FLAG = 0x01;

class VESC : public Driver {
   public:
    struct Config {
        CAN_HandleTypeDef* hcan;  // Pointer to the configured CAN handle
        uint8_t vesc_id;  // CAN ID of the VESC
    };

    VESC(const char* name);
    VESC(const char* name, const Config& config);
    ~VESC();

    bool init(const Config& config);
    bool start();

    bool setDutyCycle(float duty_cycle);
    bool setCurrent(float current_a);
    bool setRPM(int32_t rpm);
    bool setPosition(float position_rad);

   private:
    Config config;

    // static uint32_t instance_count;
    // uint32_t instance_index;

    osThreadId_t vesc_thread_id = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[VESC_THREAD_STACK_SIZE / 4]{};

    uint8_t can_filter_bank = 0xFF;
    CallbackWrapper<void(const CAN_RxHeaderTypeDef*, const uint8_t*)> receive_callback;

    bool initCAN();
    bool setCANMessageFilter();

    bool sendMessage(uint8_t vesc_id, vesc::PacketID packet_id, const uint8_t* data, uint8_t data_length);
    bool parseReceivedMessage(const CAN_RxHeaderTypeDef* header, const uint8_t* data);

    void vescThread(void* argument);

    void receiveCallback(const CAN_RxHeaderTypeDef* header, const uint8_t* data);
};