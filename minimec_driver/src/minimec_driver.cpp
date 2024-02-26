/// @file
/// @brief Forward kinematics implementation for a mecanum wheel robot.
///
/// PARAMETERS:
///     rate (double): loop frequency [hertz]
///     fl_joint_name (string): front-left joint name as defined in the robot description
///     fr_joint_name (string): front-right joint name as defined in the robot description
///     rr_joint_name (string): rear-right joint name as defined in the robot description 
///     rl_joint_name (string): rear-left joint name as defined in the robot description 
///     fl_feedback_topic (string): front-left feedback topic (published by odrive)
///     fr_feedback_topic (string): front-right feedback topic (published by odrive)
///     rr_feedback_topic (string): rear-right feedback topic (published by odrive) 
///     rl_feedback_topic (string): rear-left feedback topic (published by odrive)
///     fl_control_topic (string): front-left control topic (subscribed by odrive) 
///     fr_control_topic (string): front-right control topic (subscribed by odrive) 
///     rr_control_topic (string): rear-right control topic (subscribed by odrive) 
///     rl_control_topic (string): rear-left control topic (subscribed by odrive) 
///     fl_control_service (string): front-left service name (offered by odrive)
///     fr_control_service (string): front-right service name (offered by odrive)
///     rr_control_service (string): rear-right service name (offered by odrive)
///     rl_control_service (string): rear-left service name (offered by odrive) 

/// PUBLISHES:
///     ~/wheel_cmd (minimec_msgs::msg::WheelCommands): output wheel commands [rad/s]
/// SUBSCRIBES:
///     ~/cmd_vel (geometry_msgs::msg::Twist): input desired twist

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "minimec_msgs/msg/wheel_commands.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "minimeclib/kinematics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MinimecDriver : public rclcpp::Node
{
public:
  MinimecDriver()
  : Node("minimec_driver")
  {
    // parameters declaration
    declare_parameter("rate", 50.0);
    declare_parameter("fl_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_control_service", rclcpp::ParameterType::PARAMETER_STRING);

    // store parameters we will need
    fl_joint_name = get_parameter("fl_joint_name").as_string();
    fr_joint_name = get_parameter("fr_joint_name").as_string();
    rr_joint_name = get_parameter("rr_joint_name").as_string();
    rl_joint_name = get_parameter("rl_joint_name").as_string();

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // create publishers and subscribers
    sub_wheel_cmd_ = create_subscription<minimec_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&MinimecDriver::wheelCmdCallback, this, std::placeholders::_1));

    pub_fl_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("fl_control_topic").as_string(), 10);
    pub_fr_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("fr_control_topic").as_string(), 10);
    pub_rr_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("rr_control_topic").as_string(), 10);
    pub_rl_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("rl_control_topic").as_string(), 10);

    sub_fl_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("fl_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackFL, this, std::placeholders::_1));
    sub_fr_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("fr_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackFR, this, std::placeholders::_1));
    sub_rr_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("rr_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackRR, this, std::placeholders::_1));
    sub_rl_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("rl_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackRL, this, std::placeholders::_1));

    // create service clients
    client_fl_ = create_client<odrive_can::srv::AxisState>(get_parameter("fl_control_service").as_string());
    client_fr_ = create_client<odrive_can::srv::AxisState>(get_parameter("fr_control_service").as_string());
    client_rr_ = create_client<odrive_can::srv::AxisState>(get_parameter("rr_control_service").as_string());
    client_rl_ = create_client<odrive_can::srv::AxisState>(get_parameter("rl_control_service").as_string());

    // wait for services to become available
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
      timer_step, std::bind(&MinimecDriver::timerCallback, this));

    // create joint state publisher
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

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

      // publish joint states
      sensor_msgs::msg::JointState joint_msg;

      joint_msg.header.stamp = get_clock()->now();
      joint_msg.name = std::vector<std::string>({fl_joint_name,
            fr_joint_name, rr_joint_name,
            rl_joint_name});

      joint_msg.position = std::vector<double>({-wheel_pos.fl, -wheel_pos.fr, -wheel_pos.rr, -wheel_pos.rl});

      pub_joint_states_->publish(joint_msg);
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

  void wheelCmdCallback(const minimec_msgs::msg::WheelCommands & msg)
  {
    // translate the wheel speeds to turns/s
    wheel_speeds = minimeclib::WheelSpeeds{
      msg.fl / 2.0 / 3.14159265358979323846,
      msg.fr / 2.0 / 3.14159265358979323846,
      msg.rr / 2.0 / 3.14159265358979323846,
      msg.rl / 2.0 / 3.14159265358979323846
    };
  }

  void controllerCallbackFL(const odrive_can::msg::ControllerStatus & msg)
  {
    wheel_pos.fl = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }

  void controllerCallbackFR(const odrive_can::msg::ControllerStatus & msg)
  {
    wheel_pos.fr = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }

  void controllerCallbackRR(const odrive_can::msg::ControllerStatus & msg)
  {
    wheel_pos.rr = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }

  void controllerCallbackRL(const odrive_can::msg::ControllerStatus & msg)
  {
    wheel_pos.rl = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }

  // track if its the first run to enable odrives
  bool first_run = true;

  // number of odrives that are enabled
  int odrives_enabled{0};

  // track wheel positions and speeds
  minimeclib::WheelPositions wheel_pos;
  minimeclib::WheelSpeeds wheel_speeds;

  // publishers and subscribers
  rclcpp::Subscription<minimec_msgs::msg::WheelCommands>::SharedPtr sub_wheel_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fl_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rl_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_fl_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_fr_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_rr_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_rl_;

  // service clients
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fl_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rl_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // store joint names
  std::string fl_joint_name;
  std::string fr_joint_name;
  std::string rr_joint_name;
  std::string rl_joint_name;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimecDriver>());
  rclcpp::shutdown();
  return 0;
}
