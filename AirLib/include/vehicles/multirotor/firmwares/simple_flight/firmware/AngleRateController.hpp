
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
    AngleRateController(Params* params, const IBoardClock* clock)
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
           PidConfig<float>(params_->angle_rate_pid.p[axis], params_->angle_rate_pid.i[axis], params_->angle_rate_pid.d[axis])));
    }

    virtual void reset() override
    {
        IAxisController::reset();
        params_->angle_rate_pid.euler_rate_goal_ = Axis4r(0, 0, 0, 0);
        pid_->reset();
        output_ = TReal();
    }

    virtual void update() override
    {
        IAxisController::update();

        // The rate controller works in body rates, but goals are provided in euler rates not body rates
        // So we need to turn euler rates into body rates here using T matrix
        // in order to give the rate controller the correct target
        Axis3r euler_angles_ = state_estimator_->getAngles();
        TReal phi = euler_angles_[0];
        TReal sPhi = sin(phi);
        TReal cPhi = cos(phi);

        TReal theta = euler_angles_[1];
        TReal sTheta = sin(theta);
        TReal cTheta = cos(theta);

        params_->angle_rate_pid.euler_rate_goal_[axis_] = goal_->getGoalValue()[axis_];
        TReal phiDotGoal = params_->angle_rate_pid.euler_rate_goal_[0];
        TReal thetaDotGoal = params_->angle_rate_pid.euler_rate_goal_[1];
        TReal psiDotGoal = params_->angle_rate_pid.euler_rate_goal_[2];

        /// See section 1.3 of this Mech Eng course from MIT - equation 17
        /// https://ocw.mit.edu/courses/mechanical-engineering/2-154-maneuvering-and-control-of-surface-and-underwater-vehicles-13-49-fall-2004/lecture-notes/lec1.pdf
        TReal body_rate_goal = 0.f;
        switch (axis_) {
        case 0: // roll
            body_rate_goal = phiDotGoal - sTheta * psiDotGoal;
            break;
        case 1: // pitch
            body_rate_goal = cPhi * thetaDotGoal + sPhi * cTheta * psiDotGoal;
            break;
        case 2: // yaw
            body_rate_goal = -sPhi * thetaDotGoal + cPhi * cTheta * psiDotGoal;
            break;
        default:
            throw std::invalid_argument("axis must be 0, 1 or 2 for AngleLevelController");
        }

        pid_->setGoal(body_rate_goal);

        /// Previous code - replaced by the above
        // pid_->setGoal(goal_->getGoalValue()[axis_]); 

        pid_->setMeasured(state_estimator_->getAngularVelocity()[axis_]);
        pid_->update();

        output_ = pid_->getOutput();
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

    Params* params_;
    const IBoardClock* clock_;
    std::unique_ptr<PidController<float>> pid_;
};


} //namespace