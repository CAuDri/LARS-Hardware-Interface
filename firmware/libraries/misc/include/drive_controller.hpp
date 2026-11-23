/**
 * @file drive_controller.hpp
 *
 * @brief Drive controller for handling drive commands and modes.
 *
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

// Stack size in bytes.
constexpr size_t DRIVE_CONTROLLER_THREAD_STACK_SIZE = 4096;
constexpr size_t DRIVE_MODE_TRACE_STATE_COUNT = 5;
constexpr uint32_t DRIVE_CONTROLLER_DEFAULT_AUTONOMOUS_COMMAND_TIMEOUT_MS = 50;

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

    /**
     * @brief Autonomous motor command forwarded by ROS command nodes.
     */
    struct MotorCommand {
        enum class Mode { NONE, RPM, CURRENT };

        Mode mode = Mode::NONE;
        int32_t rpm = 0;
        float current = 0.0f;
        uint32_t timestamp_ticks = 0;
    };

    /**
     * @brief Autonomous steering command forwarded by ROS command nodes.
     */
    struct SteeringCommand {
        float angle_deg = 0.0f;
        uint32_t timestamp_ticks = 0;
    };

    struct Config {
        crsf::Channel throttle_channel = crsf::INVALID_CHANNEL;
        crsf::Channel steering_channel = crsf::INVALID_CHANNEL;
        crsf::Channel mode_switch_channel = crsf::INVALID_CHANNEL;
        uint32_t autonomous_command_timeout_ms = DRIVE_CONTROLLER_DEFAULT_AUTONOMOUS_COMMAND_TIMEOUT_MS;

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
    DriveMode getDriveMode() const { return current_drive_mode; }

    void enableAutonomousControl(bool enabled);
    bool autonomousControlEnabled() const { return autonomous_control_enabled; }

    void emergencyStop();
    bool updateMotorCommand(const MotorCommand& command);
    bool updateSteeringCommand(const SteeringCommand& command);

   private:
    enum class State { UNINITIALIZED, INITIALIZING, RUNNING, ERROR };

    State state = State::UNINITIALIZED;
    DriveMode current_drive_mode = DriveMode::IDLE;
    bool autonomous_control_enabled = false;

    const Config* config = nullptr;

    RCReceiver* rc_receiver = nullptr;
    VESC* vesc = nullptr;
    Servo* servo = nullptr;

    RCReceiver::ChannelCallback rc_callback;
    MotorCommand motor_command{};
    SteeringCommand steering_command{};

    osMutexId_t control_mutex = nullptr;
    osMutexAttr_t mutex_attributes{};
    StaticSemaphore_t mutex_control_block{};

    LightDispatcher light_dispatcher{"Tower Light Dispatcher"};

    osThreadId_t controller_thread = nullptr;
    osThreadAttr_t thread_attributes{};
    StaticTask_t thread_control_block{};
    uint32_t thread_stack[DRIVE_CONTROLLER_THREAD_STACK_SIZE / sizeof(uint32_t)]{};

    TraceStateMachineHandle_t drive_mode_machine = nullptr;
    std::array<TraceStateMachineStateHandle_t, DRIVE_MODE_TRACE_STATE_COUNT> drive_mode_trace_states{};
    bool drive_mode_trace_initialized = false;
    bool drive_mode_trace_failed = false;

    bool setState(State new_state);
    void setDriveMode(DriveMode mode);
    bool initDriveModeTracing();
    void traceDriveMode(DriveMode mode);

    void stopVehicle();
    bool handleEmergencyStop();
    bool handleModeSwitch(uint16_t channel_value);
    bool handleManualThrottle(uint16_t channel_value);
    bool handleManualSteering(uint16_t channel_value);
    bool checkAutonomousCommandTimeouts();
    bool isCommandStale(uint32_t timestamp_ticks) const;

    void controllerThread(void* argument);

    void remoteControlCallback(const crsf::ChannelData& channels);
};
