
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "AngleLevelController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"

namespace simple_flight {

class VelocityController : 
    public IAxisController,
    public IGoal  //for internal child controller
{
public:
    VelocityController(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;
		
		// PID config
		PidConfig<float> pid_config(params_->velocity_pid.p[axis], params_->velocity_pid.i[axis], params_->velocity_pid.d[axis]);
		pid_config.iterm_discount = params_->velocity_pid.iterm_discount[axis];
		pid_config.output_bias = params_->velocity_pid.output_bias[axis];

        //we will be setting goal for child controller so we need these two things
        child_mode_  = GoalMode::getUnknown();
        switch (axis_) {
        case 0: // y velocity (roll)
		case 1: // x velocity (pitch)
			pid_config.min_output = -params_->angle_level_pid.max_limit[axis];
			pid_config.max_output = params_->angle_level_pid.max_limit[axis];
			pid_.reset(new PidController<float>(clock_, pid_config));

			child_controller_.reset(new AngleLevelController(params_, clock_));

            child_mode_[axis_] = GoalModeType::AngleLevel;

            break;
        case 3: // throttle
			pid_config.min_output = params_->velocity_pid.min_throttle;
			pid_config.max_output = params_->velocity_pid.max_throttle;
			pid_config.output_bias = 0.5;
			pid_.reset(new PidController<float>(clock_, pid_config));
            
            child_controller_.reset(new PassthroughController());
			child_mode_[axis_] = GoalModeType::Passthrough;
			//passthrough controller not used - we run the PID internally in this funtion
			
			break;
        default:
            throw std::invalid_argument("axis must be 0, 1 or 3");
        }

        //initialize child controller if used
		child_controller_->initialize(axis_, this, state_estimator_);

    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        child_controller_->reset();
        child_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        //First get PID output
        const Axis3r& goal_velocity_world = Axis4r::axis4ToXyz(
            goal_->getGoalValue(), true);

		TReal yaw = state_estimator_->getAngles().yaw();
        TReal vx = goal_velocity_world.x() * cos(yaw) + goal_velocity_world.y() * sin(yaw);
        TReal vy = -goal_velocity_world.x() * sin(yaw) + goal_velocity_world.y() * cos(yaw);
        TReal vz = goal_velocity_world.z();
		// ToDo - clip to velocity limit
        Axis4r goal_velocity_new(vy, vx, 0, vz); // Note x & y swapped as these axis relate to roll & pitch outputs
        pid_->setGoal(goal_velocity_new[axis_]); //pid_->setGoal(goal_velocity_local[axis_]);

        const Axis3r& measured_velocity_world = state_estimator_->getLinearVelocity();

        vx = measured_velocity_world.x() * cos(yaw) + measured_velocity_world.y() * sin(yaw);
        vy = -measured_velocity_world.x() * sin(yaw) + measured_velocity_world.y() * cos(yaw);
        vz = measured_velocity_world.z();
        Axis4r measured_velocity_new(vy, vx, 0, vz); // Note x & y swapped as these axis relate to roll & pitch outputs
        pid_->setMeasured(measured_velocity_new[axis_]); //pid_->setMeasured(measured_velocity_local[axis_]);
        pid_->update();

        //use this to drive child controller
        switch (axis_)
        {
		case 0: // roll
		case 1: // pitch
			child_goal_[axis_] = pid_->getOutput();
            child_controller_->update();
            output_ = child_controller_->getOutput();
            break;
        case 3: //throttle
            output_ = pid_->getOutput();
            break;
        default:
            throw std::invalid_argument("axis must be 0, 1 or 3 for VelocityController");
        }

		// msr::airlib::Utils::log(msr::airlib::Utils::stringf("VC: %i\t%f\t%f\t%f",
		// 	axis_, goal_velocity_new[axis_],
		// 	measured_velocity_new[axis_], output_));
    }

    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoal ********************/
    virtual const Axis4r& getGoalValue() const override
    {
        return child_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return child_mode_;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    GoalMode child_mode_;
    Axis4r child_goal_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<IAxisController> child_controller_;

};


} //namespace