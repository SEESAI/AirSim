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

    child_mode_ = GoalMode::getUnknown();
    // The output of this controller is an angle for roll and pitch and a
    // throttle value. Throttle can directly be set as an output, but for the
    // angles we need to cascade down to angle controllers.
    switch (axis_) {
    case 0:
    case 1:
      child_controller_ = std::make_unique<AngleLevelController>(params_, clock_);
      child_mode_[axis_] = GoalModeType::AngleLevel;
      break;
    case 3:
      // we output throttle directly, technically don't need this one
      child_controller_ = std::make_unique<PassthroughController>();
      child_mode_[axis_] = GoalModeType::Passthrough;
      break;
    default:
      throw std::invalid_argument("Axis must be 0, 1 or 3. AccelerationController"
                                  "controller cannot control yaw.");
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
    static constexpr float G = 9.81f;
    TReal totalAccZ = G - az;
    float maxAccXy = totalAccZ / 1.41f;
    // We allow maximum roll or pitch angles of ~45 deg.
    ax = std::min(std::max(ax, -maxAccXy), maxAccXy);
    ay = std::min(std::max(ay, -maxAccXy), maxAccXy);
    Vector3r body_z = Vector3r(ax, ay, maxAccXy).normalized();
    // todo: add estimator instead of constant value
    static constexpr TReal hover_thrust = 0.58f;
    TReal collective_thrust = az * (hover_thrust / G) - hover_thrust;
    // project thrust to planned body attitude
    collective_thrust /= (Vector3r(0, 0, 1).dot(body_z));

    switch (axis_) {
    case 0: //+ay is +ae roll
      child_goal_[axis_] = asin(body_z.y() / body_z.z());
      child_controller_->update();
      output_ = child_controller_->getOutput();
      break;
    case 1: //+ax is -ae pitch
      child_goal_[axis_] = -asin(body_z.x() / body_z.z());
      child_controller_->update();
      output_ = child_controller_->getOutput();
      break;
    case 3: //+az is -ae thrust (NED coordinates)
      output_ = std::max(
          std::min(-collective_thrust, params_->acceleration.max_thrust),
          params_->acceleration.min_thrust);
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