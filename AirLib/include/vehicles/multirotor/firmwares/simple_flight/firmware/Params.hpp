#pragma once

#include "interfaces/CommonStructs.hpp"

namespace simple_flight {

struct Params {
public:
    static float min_armed_throttle()
    {
        static float val = 0.1f;
        return val;
    }

    //this should match up with target board
    //simulation board should respect possible values
    struct Motor {
        uint16_t motor_count = 4;
        float min_motor_output = 0;
        float max_motor_output = 1;
        //if min_armed_output too low then noise in pitch/roll can destabilize quad copter when throttle is zero
        float min_angling_throttle = Params::min_armed_throttle() / 2;
    } motor;

    struct Rc {
        uint16_t channel_count = 12;
        uint16_t read_interval_ms = 10;
        int16_t rate_level_mode_channel = 4; //corresponds to switch 0 in rc_data
        int16_t allow_api_control_channel = 5; //corresponds to switch 1 in rc_data

        //When actions such as arming/unarming, how much tolerance can be allowed in stick positions from 0 to 1?
        float action_request_tolerance = 0.1f;

        //milliseconds while sticks should stay in position
        uint64_t arm_duration = 100; 
        uint64_t disarm_duration = 100; 
        uint64_t neutral_duration = 100;
        
        Axis4<int16_t> channels = Axis4<int16_t>(0, 3, 1, 2);

        TReal max_angle_level_switch = 0.3f;

        //should be >= motor.min_angling_throttle
        float min_angling_throttle = Params::min_armed_throttle() / 1.5f;

        bool allow_api_when_disconnected = false;
        bool allow_api_always = false;

    } rc;

    struct AngleRatePid {
        //max_xxx_rate > 5 would introduce wobble/oscillations
        const float kMaxLimit = 3.5f; // (rad/s - 2.5 ~= 143 deg/s, 3.5 ~= 200deg/s)
        const float kP_RollPitch = 0.25f;
		const float kP_Yaw = 0.2f;
        const float kI = 0.0f;
        const float kD_RollPitch = 0.f;
		const float kD_Yaw = 0.f;

        Axis3r max_limit = Axis3r(kMaxLimit, kMaxLimit, kMaxLimit); //roll, pitch, yaw - in radians/sec

        //p_xxx_rate params are sensitive to gyro noise. Values higher than 0.5 would require 
        //noise filtration
        Axis4r p = Axis4r(kP_RollPitch, kP_RollPitch, kP_Yaw, 1.0f);
        Axis4r i = Axis4r(kI, kI, kI, 0.0f);
        Axis4r d = Axis4r(kD_RollPitch, kD_RollPitch, kD_Yaw, 0.0f);

        // Store of the euler_rate_goal - used to allow different axis to share the goal
        Axis4r euler_rate_goal_ = Axis4r(0,0,0,0);

    } angle_rate_pid;

    struct AngleLevelPid {
        const float pi = 3.14159265359f; //180-degrees
        const float kP_RollPitch = 6.5f;
		const float kP_Yaw = 2.8f;
        const float kI = 0.0f;
        const float kD = 0.0f;
        
        //max_pitch/roll_angle > 5.5 would produce verticle thrust that is not enough to keep vehicle in air at extremities of controls
        Axis4r max_limit = Axis4r(pi / 4.0f, pi / 4.0f, pi, 1.0f); //roll, pitch, yaw - in radians/sec

        Axis4r p = Axis4r(kP_RollPitch, kP_RollPitch, kP_Yaw, 1.0f);
        Axis4r i = Axis4r(kI, kI, kI, 0.0f);
        Axis4r d = Axis4r(kD, kD, kD, 0.0f);
    } angle_level_pid;

    struct PositionPid {
        const float kMaxLimit = 8.8E26f; //some big number like size of known universe
        const float kP = 0.25f;
        const float kI = 0.0f;
        const float kD = 0.0f;

        Axis4r max_limit = Axis4r(kMaxLimit, kMaxLimit, kMaxLimit, 1.0f); //x, y, z in meters

        Axis4r p = Axis4r(kP, kP, 0, kP);
        Axis4r i = Axis4r(kI, kI, kI, kI);
        Axis4r d = Axis4r(kD, kD, kD, kD);
    } position_pid;

    struct VelocityPid {
        const float kMinThrottle = std::min(1.0f, Params::min_armed_throttle());
        const float kMaxLimit = 6.0f; // m/s
		const float kP_XY = 0.2f;
		const float kD_XY = 0.0f; // 0.02
		const float kP_Z = 2.0f; // 0.5
        const float kI_Z = 2.0f; // 0.5

        Axis4r max_limit = Axis4r(kMaxLimit, kMaxLimit, 0, kMaxLimit); //x, y, yaw, z in meters

        Axis4r p = Axis4r(kP_XY, kP_XY, 0.0f, kP_Z); // todo why 2.0f hardcoded
        Axis4r i = Axis4r(0.0f, 0.0f, 0.0f, kI_Z);
        Axis4r d = Axis4r(kD_XY, kD_XY, 0.0f, 0.0f);

        Axis4r iterm_discount = Axis4r(1, 1, 1, 1);
        Axis4r output_bias = Axis4r(0, 0, 0, 0);
                
        //we keep min throttle higher so that if we are angling a lot, its still supported
        float min_throttle = kMinThrottle ;
        float max_throttle = 0.9f;
    } velocity_pid;

    struct Takeoff {
        float takeoff_z = -2.0f;
        //float velocity = -1.0f;
    } takeoff;

    enum class ControllerType {
        Cascade,
        Adaptive
    };

    GoalMode default_goal_mode = GoalMode::getStandardAngleMode();
    VehicleStateType default_vehicle_state = VehicleStateType::Inactive;
    uint64_t api_goal_timeout = 60; //milliseconds
    ControllerType controller_type = ControllerType::Cascade;
    bool gains_changed;
};


} //namespace