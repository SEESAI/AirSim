
#pragma once

#include "interfaces/CommonStructs.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IAxisController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"
#include <memory>
#include <string>
#include <exception>


namespace simple_flight {

class AngleRateController : public IAxisController {
public:
    AngleRateController(const Params* params, const IBoardClock* clock)
        : params_(params), clock_(clock)
    {
    }

    virtual void initialize(unsigned int axis, const IGoal* goal, const IStateEstimator* state_estimator) override
    {
        if (axis > 2)
            throw std::invalid_argument("AngleRateController only supports axis 0-2 but it was " + std::to_string(axis));

        axis_ = axis;
        goal_ = goal;
        state_estimator_ = state_estimator;

        pid_.reset(new PidController<float>(clock_,
           PidConfig<float>(params_->angle_rate_pid.p[axis], 0, params_->angle_rate_pid.d[axis], -1, 1)));
    }

    virtual void reset() override
    {
        IAxisController::reset();

        pid_->reset();
        output_ = TReal();
    }

    virtual void update() override
    {
		/// Centralised angle rate goals - to deal with cross-talk between euler and body rates
		/// NB THIS STATIC PREVENTS MORE THAN ONE DRONE IN THE SYSTEM
		/// Long term we should pass a pointer to this from the Cascade Controller, and have it live as a member there
		/// ToDo - Richard
		static Axis3r euler_rate_goal_(0,0,0);
		static Axis3r body_rate_goal_(0, 0, 0);

        IAxisController::update();

		euler_rate_goal_[axis_] = goal_->getGoalValue()[axis_];
		Axis3r euler_angles = state_estimator_->getAngles();
		Axis3r body_rate = state_estimator_->getAngularVelocity();

		// The rate controller works in body rates, but goals are provided in euler rates not body rates
		// So we need to turn euler rates into body rates here using T matrix
		// in order to give the rate controller the correct target
		TReal phi = euler_angles[0];
		TReal sPhi = sin(phi);
		TReal cPhi = cos(phi);

		TReal theta = euler_angles[1];
		TReal sTheta = sin(theta);
		TReal cTheta = cos(theta);

		// ToDo - note this could be a timestep behind, have a think about how to fix
		TReal phiDotGoal = euler_rate_goal_[0];
		TReal thetaDotGoal = euler_rate_goal_[1];
		TReal psiDotGoal = euler_rate_goal_[2];

		switch (axis_) {
		case 0: // roll
			body_rate_goal_[0] = phiDotGoal - sTheta * psiDotGoal;
			break;
		case 1: // pitch
			body_rate_goal_[1] = cPhi * thetaDotGoal + sPhi * cTheta * psiDotGoal;
			break;
		case 2: // yaw
			body_rate_goal_[2] = -sPhi * thetaDotGoal + cPhi * cTheta * psiDotGoal;
			break;
		default:
			throw std::invalid_argument("axis must be 0, 1 or 2 for AngleLevelController");
		}

        pid_->setGoal(body_rate_goal_[axis_]);
        pid_->setMeasured(body_rate[axis_]);
        pid_->update();
        
        output_ = pid_->getOutput();

		// msr::airlib::Utils::log(msr::airlib::Utils::stringf("Rate Goals: %0.2f\t%0.2f\t%0.2f Body Goals: %0.2f\t%0.2f\t%0.2f",
		// 	euler_rate_goal_[0] * 180 / 3.142, euler_rate_goal_[1] * 180 / 3.142, euler_rate_goal_ [2]*180/3.142, 
		// 	body_rate_goal_[0] * 180 / 3.142, body_rate_goal_[1] * 180 / 3.142, body_rate_goal_[2] * 180 / 3.142));

	}

    virtual TReal getOutput() override
    {
        return output_;
    }

private:
    unsigned int axis_;
    const IGoal* goal_;
    const IStateEstimator* state_estimator_;

    TReal output_;

    const Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
};


} //namespace