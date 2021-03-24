#include <cstdio>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "drive_controller_msgs/msg/swerve.hpp"
using std::placeholders::_1;

// Transforms joystick to swerve drive output 
class SwerveDriveController : public rclcpp::Node {
  public:
    SwerveDriveController() : Node("tank_drive_controller") {
      sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&SwerveDriveController::swerve_callback, this, _1));
    }

  private:
    void calculate(float forward, float strafe, float azimuth) {
      // TODO: add fieldOriented: assume gyro is in radians
      if (mFieldOriented) {
        float angle = 0.0; //getAngle();
        angle = std::fmod(angle, 2.0 * M_PI);
        const float tmp = forward * std::cos(angle) + strafe * std::sin(angle);
        strafe = strafe * std::cos(angle) - forward * std::sin(angle);
        forward = tmp;

      }
      const float a = strafe - azimuth * kLengthComponent;
      const double b = strafe + azimuth * kLengthComponent;
      const double c = forward - azimuth * kWidthComponent;
      const double d = forward + azimuth * kWidthComponent;

      ws.at(0) = std::hypot(b, d);
      ws.at(1) = std::hypot(b, c);
      ws.at(2) = std::hypot(a, d);
      ws.at(3) = std::hypot(a, c);

      // wheel azimuth
      wa.at(0) = std::atan2(b, d) * 0.5 / M_PI;
      wa.at(1) = std::atan2(b, c) * 0.5 / M_PI;
      wa.at(2) = std::atan2(a, d) * 0.5 / M_PI;
      wa.at(3) = std::atan2(a, c) * 0.5 / M_PI;

      // normalize
      const double maxWheelSpeed = std::max(std::max(ws.at(0), ws.at(1)), std::max(ws.at(2), ws.at(3)));
      if (maxWheelSpeed > 1.0)
      {
          for (int i = 0; i < 4; ++i)
          {
              ws[i] /= maxWheelSpeed;
          }
      }
    }

    void swerve_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
      calculate(msg->axes[1], msg->axes[0], msg->axes[3]);
      auto out = drive_controller_msgs::msg::Swerve();
      out.ws = ws;
      out.wa = wa;

      pub_->publish(out);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<drive_controller_msgs::msg::Swerve>::SharedPtr pub_;

    float kLengthComponent = 1.0;
    float kWidthComponent = 1.0;
    bool mFieldOriented = false;

    std::vector<float> ws{4};
    std::vector<float> wa{4};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwerveDriveController>());
  rclcpp::shutdown();
  return 0;
}
