#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "minimeclib/kinematics.hpp"

using namespace std::chrono_literals;

class VelocityParser : public rclcpp::Node
{
public:
  VelocityParser()
  : Node("velocity_parser")
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
      "cmd_vel", 10, std::bind(&VelocityParser::cmdVelCallback, this, std::placeholders::_1));

    pub_fl_ = create_publisher<odrive_can::msg::ControlMessage>("odrive_axis3/control_message", 10);
    pub_fr_ = create_publisher<odrive_can::msg::ControlMessage>("odrive_axis0/control_message", 10);
    pub_rr_ = create_publisher<odrive_can::msg::ControlMessage>("odrive_axis2/control_message", 10);
    pub_rl_ = create_publisher<odrive_can::msg::ControlMessage>("odrive_axis1/control_message", 10);

    // create service clients
    client_fl_ = create_client<odrive_can::srv::AxisState>("odrive_axis3/request_axis_state");
    client_fr_ = create_client<odrive_can::srv::AxisState>("odrive_axis0/request_axis_state");
    client_rr_ = create_client<odrive_can::srv::AxisState>("odrive_axis2/request_axis_state");
    client_rl_ = create_client<odrive_can::srv::AxisState>("odrive_axis1/request_axis_state");

    while (!client_fl_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_fr_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_rr_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    while (!client_rl_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&VelocityParser::timerCallback, this));

  }

private:
  void timerCallback()
  {
    if (first_run) {
      enableOdrives();
      first_run = false;
    } else if (odrives_enabled < 4) {
      RCLCPP_INFO(this->get_logger(), "odrives enabled %d", odrives_enabled);
    } else {
      auto msg = odrive_can::msg::ControlMessage{};
      msg.control_mode = 2;
      msg.input_mode = 1;
      msg.input_pos = 0.0;
      msg.input_torque = 0.0;

      // front left
      msg.input_vel = -wheel_speeds.fl;
      pub_fl_->publish(msg);

      // front right
      msg.input_vel = wheel_speeds.fr;
      pub_fr_->publish(msg);

      // rear right
      msg.input_vel = wheel_speeds.rr;
      pub_rr_->publish(msg);

      // rear left
      msg.input_vel = -wheel_speeds.rl;
      pub_rl_->publish(msg);
    }
  }

  void enableOdrives()
  {
    auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
    request->axis_requested_state = 8;

    auto callback = [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Received response from odrive");
        odrives_enabled += 1;
      };

    client_fl_->async_send_request(request, callback);

    client_fr_->async_send_request(request, callback);

    client_rr_->async_send_request(request, callback);

    client_rl_->async_send_request(request, callback);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist & msg)
  {
    wheel_speeds = mecanum_drive.IKin(msg.angular.z, msg.linear.x, msg.linear.y);
  }

  minimeclib::WheelSpeeds wheel_speeds;

  double r;
  double w;
  double l;
  int odrives_enabled{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fl_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rl_;

  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fl_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rl_;

  minimeclib::MecanumDrive mecanum_drive;

  rclcpp::TimerBase::SharedPtr timer_;

  bool first_run = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityParser>());
  rclcpp::shutdown();
  return 0;
}
