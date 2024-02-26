/// @file
/// @brief Forward kinematics implementation for a mecanum wheel robot.
///
/// PARAMETERS:
///     w (double): track wheel [meters]
///     r (double): wheel radius [meters]
///     l (double): wheel base [meters]
///     rate (double): loop frequency [hertz]
/// PUBLISHES:
///     ~/wheel_cmd (minimec_msgs::msg::WheelCommands): output wheel commands [rad/s]
/// SUBSCRIBES:
///     ~/cmd_vel (geometry_msgs::msg::Twist): input desired twist

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "minimeclib/kinematics.hpp"
#include "minimec_msgs/msg/wheel_commands.hpp"

using namespace std::chrono_literals;

class Kinematics : public rclcpp::Node
{
public:
  Kinematics()
  : Node("kinematics")
  {
    // parameters declaration
    declare_parameter("w", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("r", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("l", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("rate", 50.0);

    r = get_parameter("r").as_double();
    w = get_parameter("w").as_double();
    l = get_parameter("l").as_double();

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // instance MecanumDrive class
    mecanum_drive = minimeclib::MecanumDrive{w, l, r};

    // create publishers and subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Kinematics::cmdVelCallback, this, std::placeholders::_1));

    pub_wheel_cmd_ = create_publisher<minimec_msgs::msg::WheelCommands>("wheel_cmd", 10);
    
    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&Kinematics::timerCallback, this));

  }

private:
  void timerCallback()
  {
    pub_wheel_cmd_->publish(wheel_cmd);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist & msg)
  {
    auto wheel_speeds_rads = mecanum_drive.IKin(msg.angular.z, msg.linear.x, msg.linear.y); // we receive these in rad/s
    wheel_cmd.fl = wheel_speeds_rads.fl;
    wheel_cmd.fr = wheel_speeds_rads.fr;
    wheel_cmd.rr = wheel_speeds_rads.rr;
    wheel_cmd.rl = wheel_speeds_rads.rl;
  }

  double r;
  double w;
  double l;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<minimec_msgs::msg::WheelCommands>::SharedPtr pub_wheel_cmd_;

  minimec_msgs::msg::WheelCommands wheel_cmd;

  minimeclib::MecanumDrive mecanum_drive;

  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Kinematics>());
  rclcpp::shutdown();
  return 0;
}
