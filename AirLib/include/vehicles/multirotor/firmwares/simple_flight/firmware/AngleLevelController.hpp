
#pragma once

#include "interfaces/IUpdatable.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "AngleRateController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"
#include <string>
#include <exception>


namespace simple_flight {

class AngleLevelController : 
    public IAxisController,
    public IGoal  //for internal rate controller
{
public:
    AngleLevelController(const Params* params, const IBoardClock* clock = nullptr)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        if (axis > 2)
            throw std::invalid_argument("AngleLevelController only supports axis 0-2 but it was " + std::to_string(axis));

        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        //initialize level PID
        pid_.reset(new PidController<float>(clock_,
            PidConfig<float>(params_->angle_level_pid.p[axis], 0, 0, 
				-params_->angle_rate_pid.max_limit[axis], params_->angle_rate_pid.max_limit[axis])));

        //initialize rate controller
		body_rate_controller_.reset(new AngleRateController(params_, clock_));
		body_rate_controller_->initialize(axis, this, state_estimator_);

        //we will be setting goal for rate controller so we need these two things
        rate_mode_  = GoalMode::getUnknown();
        rate_mode_[axis] = GoalModeType::AngleRate;
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
		body_rate_controller_->reset();
		euler_rate_goal_ = Axis4r();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        //get response of level PID
        TReal goal_euler_angle = goal_->getGoalValue()[axis_];
        TReal measured_euler_angle = state_estimator_->getAngles()[axis_];

		// adjust to smallest distance (to deal with angle wrap in yaw)
        adjustToMinDistanceAngles(measured_euler_angle, goal_euler_angle);
            
        pid_->setGoal(goal_euler_angle);
        pid_->setMeasured(measured_euler_angle);
        pid_->update();

		//get the required euler rates as output
		euler_rate_goal_[axis_] = pid_->getOutput();

		// update the body rate controller
        body_rate_controller_->update();

        //rate controller's output is final output
        output_ = body_rate_controller_->getOutput();

		// msr::airlib::Utils::log(msr::airlib::Utils::stringf("Output: %i\t%f\t%f",
		// 	axis_, goal_euler_angle * 180 / 3.142, measured_euler_angle*180/3.142));
    }

    
    virtual TReal getOutput() override
    {
        return output_;
    }

    /********************  IGoal ********************/
    virtual const Axis4r& getGoalValue() const override
    {
        return euler_rate_goal_;
    }

    virtual const GoalMode& getGoalMode() const  override
    {
        return rate_mode_;
    }

private:
    static void adjustToMinDistanceAngles(TReal& angle1, TReal& angle2)
    {
        static constexpr TReal TwoPi = 2 * M_PIf;

        //first make sure both angles are restricted from -360 to +360
        angle1 = static_cast<TReal>(std::fmod(angle1, TwoPi));
        angle2 = static_cast<TReal>(std::fmod(angle2, TwoPi));

        //now make sure both angles are restricted from 0 to 360
        if (angle1 < 0)
            angle1 = TwoPi + angle1;
        if (angle2 < 0)
            angle2 = TwoPi + angle2;

        //measure distance between two angles
        auto dist = angle1 - angle2;

        //if its > 180 then invert first angle
        if (dist > M_PIf)
            angle1 = angle1 - TwoPi;
        //if two much on other side then invert second angle
        else if (dist < -M_PIf)
            angle2 = angle2 - TwoPi;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    GoalMode rate_mode_;
	Axis4r euler_rate_goal_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
    std::unique_ptr<AngleRateController> body_rate_controller_;

};


} //namespace