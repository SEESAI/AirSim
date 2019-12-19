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
        float min_motor_output = 0.0f; // don't change these - they are hard coded in the physics to be between 0-1
        float max_motor_output = 1.0f; // don't change these - they are hard coded in the physics to be between 0-1
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
        const float kMaxLimit = 200.0f * 3.1416 / 180; // Airsim Default = 2.5f, but then changed way limits work
        Axis3r max_limit = Axis3r(kMaxLimit, kMaxLimit, kMaxLimit); //roll, pitch, yaw - in radians/sec

        //p_xxx_rate params are sensitive to gyro noise. Values higher than 0.5 would require noise filtration
        Axis3r p = Axis3r(0.25f, 0.25f, 0.2f); //roll, pitch, yaw gains, Airsim Default = 0.25f
		Axis3r d = Axis3r(0.003f, 0.003f, 0.0f); //roll, pitch, yaw gains
	} angle_rate_pid;

    struct AngleLevelPid {
        const float pi = 3.14159265359f; //180-degrees
        
        //max_pitch/roll_angle > pi/5.5 would produce versicle thrust that is not enough to keep vehicle in air at extremities of controls
		Axis3r max_limit = Axis3r(pi / 4.0f, pi / 4.0f, pi); //roll, pitch, yaw - in radians

        Axis3r p = Axis3r(6.5f, 6.5f, 2.8f); //roll, pitch, yaw
    } angle_level_pid;


	struct VelocityPid {
		Axis4r max_limit = Axis4r(6.0f, 6.0f, 0, 6.0f); //x, y, <not used>, z in meters

		Axis4r p = Axis4r(0.2, -0.2, 0, -0.5f); //roll, pitch, <not used>, throttle, airsim default 0.2f
		Axis4r i = Axis4r(0, 0, 0, -0.5f); //roll, pitch, <not used>, throttle
		Axis4r d = Axis4r(0.02, -0.02, 0, 0); //roll, pitch, <not used>, throttle
		Axis4r iterm_discount = Axis4r(1, 1, 1, 1); //roll, pitch, <not used>, throttle, throttle was 0.9999f
		Axis4r output_bias = Axis4r(0, 0, 0, 0);

		//we keep min throttle higher so that if we are angling a lot, its still supported
		float min_throttle = 0.1f; // std::min(1.0f, Params::min_armed_throttle() * 3.0f);
		float max_throttle = 0.9f; // stops saturation

	} velocity_pid;
	
	struct PositionPid {
        const float kMaxLimit = 8.8E26f; //some big number like size of known universe
        Axis4r max_limit = Axis4r(kMaxLimit, kMaxLimit, 0, 1.0f); //x, y, <not used>, z in meters

        Axis4r p = Axis4r( 0.25f,  0.25f, 0, 0.25f);
    } position_pid;

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
};


} //namespace