#pragma once

#include "AngleLevelController.hpp"
#include "Params.hpp"
#include "PidController.hpp"
#include "common/common_utils/Utils.hpp"
#include "interfaces/CommonStructs.hpp"
#include "interfaces/IAxisController.hpp"
#include "interfaces/IBoardClock.hpp"
#include "interfaces/IGoal.hpp"
#include "interfaces/IUpdatable.hpp"

namespace simple_flight {

class AccelerationController : public IAxisController,
                           public IGoal // for internal child controller
{
public:
  AccelerationController(Params *params, const IBoardClock *clock = nullptr)
      : params_(params), clock_(clock) {}

  virtual void initialize(unsigned int axis, const IGoal *goal,
                          const IStateEstimator *state_estimator) override {
    axis_ = axis;
    goal_ = goal;
    state_estimator_ = state_estimator;

    // we will be setting goal for child controller so we need these two things
    child_mode_ = GoalMode::getUnknown();
    switch (axis_) {
    case 0:
      child_controller_.reset(new AngleLevelController(params_, clock_));
      child_mode_[axis_] = GoalModeType::AngleLevel; // ay = roll
      break;
    case 1:
      child_controller_.reset(new AngleLevelController(params_, clock_));
      child_mode_[axis_] = GoalModeType::AngleLevel; // ax = - pitch
      break;
    case 2:
      // we control yaw
      throw std::invalid_argument(
          "axis must be 0, 1 or 3 but it was " + std::to_string(axis_) +
          " because yaw cannot be controlled by AccelerationController");
    case 3:
      // not really required
      // output of parent controller is -1 to 1 which
      // we will transform to 0 to 1
      child_controller_.reset(new PassthroughController());
      child_mode_[axis_] = GoalModeType::Passthrough;
      break;
    default:
      throw std::invalid_argument("axis must be 0 to 2");
    }

    // initialize child controller
    child_controller_->initialize(axis_, this, state_estimator_);
  }

  virtual void reset() override {
    IAxisController::reset();

    child_controller_->reset();
    child_goal_ = Axis4r();
    output_ = TReal();
  }

  virtual void update() override {
    IAxisController::update();

    // Convert acceleration to linearised body frame
    const Axis3r &goal_acc_world =
        Axis4r::axis4ToXyz(goal_->getGoalValue(), true);
    TReal yaw = state_estimator_->getAngles().yaw();
    TReal ax = goal_acc_world.x() * cos(yaw) + goal_acc_world.y() * sin(yaw);
    TReal ay = -goal_acc_world.x() * sin(yaw) + goal_acc_world.y() * cos(yaw);
    TReal az = goal_acc_world.z();

    // Convert acceleration setpoint to thrust vector
    Vector3r body_z = Vector3r( ax, ay, 9.81f).normalized();
    // todo: add estimator instead of constant value
    static constexpr TReal hover_thrust = 0.6f;
    TReal collective_thrust = az * (hover_thrust / 9.81f) - hover_thrust;
    // project thrust to planned body attitude
    collective_thrust /= (Vector3r(0, 0, 1).dot(body_z));

    // use this to drive child controller
    switch (axis_) {
    case 0: //+ay is +ae roll
      child_goal_[axis_] = asin(body_z.y() / body_z.z());
      child_controller_->update();
      output_ = child_controller_->getOutput();
      break;
    case 1: //+ax is -ve pitch
      child_goal_[axis_] = -asin(body_z.x() / body_z.z());
      child_controller_->update();
      output_ = child_controller_->getOutput();
      break;
    case 3: //+az is -ae thrust (NED coordinates)
      output_ = std::clamp(-collective_thrust, params_->acceleration.min_thrust, params_->acceleration.max_thrust);
      break;
    default:
      throw std::invalid_argument(
          "axis must be 0, 1 or 3 for VelocityController");
    }
  }

  virtual TReal getOutput() override { return output_; }

  /********************  IGoal ********************/
  virtual const Axis4r &getGoalValue() const override { return child_goal_; }

  virtual const GoalMode &getGoalMode() const override { return child_mode_; }

private:

  unsigned int axis_;
  const IGoal *goal_;
  const IStateEstimator *state_estimator_;

  GoalMode child_mode_;
  Axis4r child_goal_;

  TReal output_;

  Params *params_;
  const IBoardClock *clock_;
  std::unique_ptr<IAxisController> child_controller_;
};

} // namespace simple_flight