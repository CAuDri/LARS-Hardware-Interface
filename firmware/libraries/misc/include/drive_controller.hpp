/**
 * @file drive_controller.hpp
 *
 * @brief Drive controller node for handling drive commands and modes.
 */
#pragma once

#include <array>

#include "cmsis_os.h"
#include "rc_receiver.hpp"
#include "servo.hpp"
#include "trcRecorder.h"
#include "vesc.hpp"
#include "light.hpp"
#include "light_dispatcher.hpp"

constexpr size_t DRIVE_CONTROLLER_THREAD_STACK_SIZE = 1024;
constexpr size_t DRIVE_MODE_TRACE_STATE_COUNT = 5;

enum class NodeState { UNINITIALIZED, INITIALIZING, RUNNING, ERROR };

class DriveController {
   public:
   /**
    * @brief Possible drive modes for the vehicle
    * 
    * @param IDLE The vehicle is idle and not responding to commands
    * @param MANUAL The vehicle is under manual control via RC receiver
    * @param AUTONOMOUS The vehicle is under autonomous control
    * @param MANDATORY_STOP The vehicle is executing a mandatory stop procedure
    * @param EMERGENCY_STOP The vehicle is executing an emergency stop procedure
    */
    enum class DriveMode { IDLE, MANUAL, AUTONOMOUS, MANDATORY_STOP, EMERGENCY_STOP };

    struct Config {
        crsf::Channel throttle_channel = crsf::INVALID_CHANNEL;
        crsf::Channel steering_channel = crsf::INVALID_CHANNEL;
        // crsf::Channel deadman_switch_channel = crsf::INVALID_CHANNEL;
        crsf::Channel mode_switch_channel = crsf::INVALID_CHANNEL;

        Color manual_mode_color = COLOR_BLUE;
        Color autonomous_mode_color = Color(0, 100, 0);
        Color mandatory_stop_color = COLOR_ORANGE;
        Color emergency_stop_color = COLOR_RED;

        osPriority_t thread_priority = osPriorityNormal;
    };

    DriveController();
    DriveController(const Config& config, RCReceiver& rc_receiver, VESC& vesc, Servo& servo, Light& status_light);
    ~DriveController();

    bool init(const Config& config, RCReceiver& rc_receiver, VESC& vesc, Servo& servo, Light& status_light);
    bool start();
    bool restart();

    bool changeDriveMode(DriveMode mode);
    DriveMode getDriveMode() { return current_drive_mode; }

    void emergencyStop();

   private:
    NodeState state = NodeState::UNINITIALIZED;
    DriveMode current_drive_mode = DriveMode::IDLE;

    const Config* config = nullptr;

    RCReceiver* rc_receiver = nullptr;
    VESC* vesc = nullptr;
    Servo* servo = nullptr;

    RCReceiver::ChannelCallback rc_callback;
    uint32_t last_rc_update_timestamp = 0;

    LightDispatcher light_dispatcher{"Tower Light Dispatcher"};

    osThreadId_t controller_thread = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[DRIVE_CONTROLLER_THREAD_STACK_SIZE]{};

    TraceStateMachineHandle_t drive_mode_machine = nullptr;
    std::array<TraceStateMachineStateHandle_t, DRIVE_MODE_TRACE_STATE_COUNT> drive_mode_trace_states{};
    bool drive_mode_trace_initialized = false;
    bool drive_mode_trace_failed = false;

    bool setState(NodeState new_state);
    void setDriveMode(DriveMode mode);
    bool initDriveModeTrace();
    void traceDriveMode(DriveMode mode);

    void stopVehicle();
    bool handleEmergencyStop();
    bool handleModeSwitch(uint16_t channel_value);
    bool handleThrottle(uint16_t channel_value);
    bool handleSteering(uint16_t channel_value);

    void controllerThread(void* argument);

    void remoteControlCallback(const crsf::ChannelData& channels);
};
