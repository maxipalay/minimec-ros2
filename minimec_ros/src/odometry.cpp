#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "minimeclib/kinematics.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;


class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // parameters declaration
    declare_parameter("w", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("r", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("l", rclcpp::ParameterType::PARAMETER_DOUBLE);
    declare_parameter("rate", 50.0);
    declare_parameter("odom_id", "odom");
    declare_parameter("body_id", "base_footprint");

    r = get_parameter("r").as_double();
    w = get_parameter("w").as_double();
    l = get_parameter("l").as_double();

    odom_id = get_parameter("odom_id").as_string();
    body_id = get_parameter("body_id").as_string();

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // initialize odom data
    odom_msg.header.frame_id = odom_id;
    odom_msg.child_frame_id = body_id;
    
    odom_transform.header.frame_id = odom_id;
    odom_transform.child_frame_id = body_id;

    // instance diffDrive class
    mecanum_drive = minimeclib::MecanumDrive{w, l, r};

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // create publishers and subscribers
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::jointStateCallback, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&Odometry::timerCallback, this));

  }

private:
  void timerCallback()
  {
    pub_odom_->publish(odom_msg);
    tf_broadcaster_->sendTransform(odom_transform);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState & msg)
  {
    auto time_now = get_clock()->now();
    // we should be using the names provided
    // to find out the indices of those in the vector
    
    if (!first_joints_cb) {

      auto relative_transform = mecanum_drive.FKin(minimeclib::WheelPositions{msg.position.at(0), -msg.position.at(1), -msg.position.at(2), msg.position.at(3)});
      auto new_transform = prev_transform * relative_transform;

      auto dt = (time_now - time_last_joint_data).seconds();
      odom_msg.header.stamp = time_now;
      odom_msg.pose.pose.position.x = new_transform.getOrigin().x();
      odom_msg.pose.pose.position.y = new_transform.getOrigin().y();
      
      odom_msg.pose.pose.orientation = tf2::toMsg(new_transform.getRotation().normalized());
      
      odom_msg.twist.twist.linear.x = (new_transform.getOrigin().x() - prev_transform.getOrigin().x()) /
        (dt);
      odom_msg.twist.twist.linear.y = (new_transform.getOrigin().y() - prev_transform.getOrigin().y()) /
        (dt);

      auto new_transform_rotation = new_transform.getRotation().normalized();
      auto prev_transform_rotation = prev_transform.getRotation().normalized();
      
      double new_yaw, new_pitch, new_roll;
      tf2::getEulerYPR(new_transform_rotation, new_yaw, new_pitch, new_roll);

      double prev_yaw, prev_pitch, prev_roll;
      tf2::getEulerYPR(prev_transform_rotation, prev_yaw, prev_pitch, prev_roll);

      odom_msg.twist.twist.angular.z = (new_yaw - prev_yaw) / (dt);
      
      //pub_odom_->publish(odom_msg);

      odom_transform.header.stamp = time_now;
      odom_transform.transform.translation.x = new_transform.getOrigin().x();
      odom_transform.transform.translation.y = new_transform.getOrigin().y();
      odom_transform.transform.rotation = tf2::toMsg(new_transform.getRotation().normalized());
      
      //tf_broadcaster_->sendTransform(odom_transform);
      prev_transform = new_transform;
    } else {
      first_joints_cb = false;
      mecanum_drive.setWheelOffsets(minimeclib::WheelPositions{msg.position.at(0), -msg.position.at(1), -msg.position.at(2), msg.position.at(3)});
    }
    time_last_joint_data = time_now;
  }

  bool first_joints_cb = true;
  double w, r, l;
  std::string body_id;
  std::string odom_id;
  minimeclib::MecanumDrive mecanum_drive = minimeclib::MecanumDrive{};
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg{};
  geometry_msgs::msg::TransformStamped odom_transform;
  
  rclcpp::Time time_last_joint_data;
  tf2::Transform prev_transform{tf2::Quaternion(0,0,0,1)};
  
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}