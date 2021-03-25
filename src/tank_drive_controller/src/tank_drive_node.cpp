#include <cstdio>
#include <array>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "drive_controller_msgs/msg/tank.hpp"
using std::placeholders::_1;

// Transforms joystick to Tank Drive Left/Right output 

class TankDriveController : public rclcpp::Node {
  public:
    TankDriveController() : Node("tank_drive_controller") {
      sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&TankDriveController::tank_callback, this, _1));
      pub_ = this->create_publisher<drive_controller_msgs::msg::Tank>("tank_output", 10);
    }

  private:
    std::array<float, 2> calculate(float forward, float strafe) {
      forward = handleDeadband(forward, kDeadband);
      strafe = handleDeadband(strafe, kDeadband);

      float angle = std::fabs(atan2(forward, strafe));
      bool quickturn = ( angle > M_PI - kQuickTurnAngle ) || ( angle < kQuickTurnAngle ); 

      float negInertia = strafe - mOldStrafe;
      mOldStrafe = strafe;

      float denominator = std::sin(M_PI / 2.0 * kWheelNonLinearity);

      strafe = std::sin(M_PI / 2.0 * kWheelNonLinearity * strafe) / denominator;
      strafe = std::sin(M_PI / 2.0 * kWheelNonLinearity * strafe) / denominator;
    
      float negInertiaPower = negInertia * kNegInertiaScalar;
      mNegInertiaAccumulator += negInertiaPower;

      float leftOut, rightOut, overPower, angularPower, linearPower;

      strafe += mNegInertiaAccumulator;
      if (mNegInertiaAccumulator > 1.0) {
        mNegInertiaAccumulator -= 1.0;
      } else if (mNegInertiaAccumulator < -1.0) {
        mNegInertiaAccumulator += 1.0;
      } else {
        mNegInertiaAccumulator = 0.0;
      }

      linearPower = forward;

      if (quickturn) {
        if (std::fabs(linearPower) < kQuickStopDeadband) {
          mQuickStopAccumulator = (1 - kQuickStopWeight) * mQuickStopAccumulator + kQuickStopWeight * (strafe > 1.0? 1.0:strafe<-1.0?-1.0:strafe) * kQuickStopScalar;
        }
        overPower = 1.0;
        angularPower = strafe;
      } else {
        overPower = 0.0;
        angularPower = std::fabs(forward) * strafe * kSensitivity - mQuickStopAccumulator;
        if (mQuickStopAccumulator > 1.0) {
          mQuickStopAccumulator -= 1.0;
        } else if (mQuickStopAccumulator < -1.0) {
          mQuickStopAccumulator += 1.0;
        } else {
          mQuickStopAccumulator = 0.0;
        }
      }
      

      rightOut = leftOut = linearPower;
      leftOut += angularPower;
      rightOut -= angularPower;

      if (leftOut > 1.0) {
        rightOut -= overPower * (leftOut - 1.0);
        leftOut = 1.0;
      } else if (rightOut > 1.0) {
        leftOut -= overPower * (rightOut - 1.0);
        rightOut = 1.0;
      } else if (leftOut < -1.0) {
        rightOut += overPower * (-1.0 - leftOut);
        leftOut = -1.0;
      } else if (rightOut < -1.0) {
        leftOut += overPower * (-1.0 - rightOut);
        rightOut = -1.0;
      }

      // Flip output of quickturn and forward
      if (quickturn || forward > 0.0 ) {
        double tmp = leftOut;
        leftOut = rightOut;
        rightOut = tmp;
      }

      std::array<float, 2> output;
      output[0] = leftOut;
      output[1] = rightOut;
      return output;
    }

    double handleDeadband(double val, double deadband) {
      return (std::fabs(val) > std::fabs(deadband)) ? val : 0.0;
    }

    void tank_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
      std::array<float, 2> tmp = calculate(msg->axes[1], msg->axes[0]);
      auto out = drive_controller_msgs::msg::Tank();
      out.left = tmp[0];
      out.right = tmp[1];

      pub_->publish(out);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<drive_controller_msgs::msg::Tank>::SharedPtr pub_;

    const float kDeadband = 0.05;
    float kQuickTurnAngle = M_PI/6;
    float mOldStrafe = 0.0;
    float mQuickStopAccumulator = 0.0;
    float mNegInertiaAccumulator = 0.0;
    
    //Determines how fast transverses sine curve
    const float kWheelNonLinearity = 0.65;
    const float kNegInertiaScalar = 4.0;

    const float kSensitivity = 0.65;
    const float kQuickStopDeadband = 0.5;
    const float kQuickStopWeight = 0.1;
    const float kQuickStopScalar = 5.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TankDriveController>());
  rclcpp::shutdown();
  return 0;
}
