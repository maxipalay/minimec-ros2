/// @file
/// @brief Track a reference trajectory
///
/// PARAMETERS:
///     rate (double): control loop rate (Hz)
/// PUBLISHERS:
///     /cmd_vel (geometry_msgs::msg::Twist): output velocity commands
/// SUBSCRIBERS:
///     /odom (sensor_msgs::msg::JointState): robot joint's state
///     /plan (nav_msgs::msg::Path): input reference trajectory

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <armadillo>
#include <cmath>

using namespace std::chrono_literals;

class TrajectoryTracker : public rclcpp::Node
{
public:
  TrajectoryTracker()
  : Node("trajectory_tracker")
  {
    // parameters declaration
    declare_parameter("rate", 50.0);

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // create publishers and subscribers
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // subscriber to odometry (feedback)
    sub_odom_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&TrajectoryTracker::odomCallback, this, std::placeholders::_1));

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&TrajectoryTracker::timerCallback, this));

    // qos for path
    auto qos_ = rclcpp::SystemDefaultsQoS{};
    qos_.reliable();

    // path subscriber
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      "plan", qos_,
      std::bind(&TrajectoryTracker::planCallback, this, std::placeholders::_1));


  }

private:
  void planCallback(const nav_msgs::msg::Path & msg)
  {
    traj = msg;
    received_path = true;

  }
  void timerCallback()
  {

    if (received_path && counter < traj.poses.size() - 1) {

      // current configuration
      auto current_config = tf2::Transform();
      tf2::fromMsg(odom_msg.pose.pose, current_config);

      // reference configuration
      auto reference_config = tf2::Transform();
      tf2::fromMsg(traj.poses.at(counter).pose, reference_config);

      // next reference configuration
      auto next_config = tf2::Transform();
      tf2::fromMsg(traj.poses.at(counter + 1).pose, next_config);

      // feedback control law
      // get the rotation matrix of the transform

      // get the transform that takes us from our current position to the desired position
      auto tf_diff = current_config.inverse() * reference_config;

      // make this an armadillo matrix so we can operate easier
      auto rot = tf_diff.getBasis();


      auto trans = tf_diff.getOrigin();
      arma::mat tf_diff_mat = {
        {rot.getRow(0).getX(), rot.getRow(0).getY(), rot.getRow(0).getZ(), trans.getX()},
        {rot.getRow(1).getX(), rot.getRow(1).getY(), rot.getRow(1).getZ(), trans.getY()},
        {rot.getRow(2).getX(), rot.getRow(2).getY(), rot.getRow(2).getZ(), trans.getZ()},
        {0.0, 0.0, 0.0, 1.0}
      };

      // logaritm of tf_diff_mat
      arma::mat log{};

      arma::Mat<double> rot_mat = tf_diff_mat.submat(0, 0, 2, 2);

      arma::vec trans_vec = {trans[0], trans[1], trans[2]};

      // calculate log3

      arma::mat log3;

      double acosinput = (arma::trace(rot_mat) - 1.0) / 2.0;
      if (acosinput >= 1.0) {
        log3 = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      } else if (acosinput <= -1.0) {
        arma::vec omg;
        if (1.0 + rot_mat(2, 2) > 0.0001 || -1.0 - rot_mat(2, 2) < -0.0001) {
          arma::vec vec;
          vec = {rot_mat(0, 2), rot_mat(1, 2), 1 + rot_mat(2, 2)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(2, 2)))) * vec;
        } else if (1.0 + rot_mat(1, 1) > 0.0001 || -1.0 - rot_mat(1, 1) < -0.0001) {
          arma::vec vec;
          vec = {rot_mat(0, 1), 1.0 + rot_mat(1, 1), rot_mat(2, 1)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(1, 1)))) * vec;
        } else {
          arma::vec vec;
          vec = {rot_mat(0, 0) + 1.0, rot_mat(1, 0), rot_mat(2, 0)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(1, 1)))) * vec;
        }
        arma::mat m = 3.14159 * omg;
        log3 = {{0.0, -m[2], m[1]}, {m[2], 0.0, -m[0]}, {-m[1], m[0], 0.0}};

      } else {
        double theta = std::acos(acosinput);
        log3 = theta / 2.0 / std::sin(theta) * ( (rot_mat - rot_mat.t()));
      }

      if (log3.is_zero(0.01)) {
        log = {{0.0, 0.0, 0.0, tf_diff_mat(0, 3)},
          {0.0, 0.0, 0.0, tf_diff_mat(1, 3)},
          {0.0, 0.0, 0.0, tf_diff_mat(2, 3)},
          {0.0, 0.0, 0.0, 0.0}};
      } else {
        auto theta = std::acos((trace(rot_mat) - 1.0) / 2.0);
        arma::mat eye;
        eye.eye(3, 3);

        arma::vec result =
          (eye - log3 / 2.0 + (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2.0) * log3 * log3 /
          theta) * trans_vec;

        log = {{log3(0, 0), log3(0, 1), log3(0, 2), result[0]},
          {log3(1, 0), log3(1, 1), log3(1, 2), result[1]},
          {log3(2, 0), log3(2, 1), log3(2, 2), result[2]},
          {0.0, 0.0, 0.0, 0.0}};
      }

      // finish logarithm

      arma::Col<double> x_err = {log(2, 1), log(0, 2), log(1, 0), log(0, 3), log(1, 3), log(2, 3)};

      // get the transform that takes us from our desired position to the next desired position
      tf_diff = reference_config.inverse() * next_config;

      // make this an armadillo matrix so we can operate easier
      rot = tf_diff.getBasis();
      trans = tf_diff.getOrigin();
      tf_diff_mat = {
        {rot.getRow(0).getX(), rot.getRow(0).getY(), rot.getRow(0).getZ(), trans.getX()},
        {rot.getRow(1).getX(), rot.getRow(1).getY(), rot.getRow(1).getZ(), trans.getY()},
        {rot.getRow(2).getX(), rot.getRow(2).getY(), rot.getRow(2).getZ(), trans.getZ()},
        {0.0, 0.0, 0.0, 1.0}
      };

      tf_diff_mat *= 50.0;

      // logaritm of tf_diff_mat

      rot_mat = {{rot[0][0], rot[0][1], rot[0][2]},
        {rot[1][0], rot[1][1], rot[1][2]},
        {rot[2][0], rot[2][1], rot[2][2]}};

      trans_vec = {trans[0], trans[1], trans[2]};

      // calculate log3

      acosinput = (arma::trace(rot_mat) - 1.0) / 2.0;
      if (acosinput >= 1.0) {
        log3 = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
      } else if (acosinput <= -1.0) {
        arma::vec omg;
        if (1.0 + rot_mat(2, 2) > 0.0001 || -1.0 - rot_mat(2, 2) < -0.0001) {
          arma::vec vec;
          vec = {rot_mat(0, 2), rot_mat(1, 2), 1 + rot_mat(2, 2)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(2, 2)))) * vec;
        } else if (1.0 + rot_mat(1, 1) > 0.0001 || -1.0 - rot_mat(1, 1) < -0.0001) {
          arma::vec vec;
          vec = {rot_mat(0, 1), 1.0 + rot_mat(1, 1), rot_mat(2, 1)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(1, 1)))) * vec;
        } else {
          arma::vec vec;
          vec = {rot_mat(0, 0) + 1.0, rot_mat(1, 0), rot_mat(2, 0)};
          omg = (1.0 / std::sqrt(2.0 * (1 + rot_mat(1, 1)))) * vec;
        }
        arma::mat m = 3.14159 * omg;
        log3 = {{0.0, -m[2], m[1]}, {m[2], 0.0, -m[0]}, {-m[1], m[0], 0.0}};

      } else {
        double theta = std::acos(acosinput);
        log3 = theta / 2.0 / std::sin(theta) * ( (rot_mat - rot_mat.t()));
      }

      if (log3.is_zero(0.0000000000001)) {
        log = {{0.0, 0.0, 0.0, tf_diff_mat(0, 3)},
          {0.0, 0.0, 0.0, tf_diff_mat(1, 3)},
          {0.0, 0.0, 0.0, tf_diff_mat(2, 3)},
          {0.0, 0.0, 0.0, 0.0}};
      } else {
        auto theta = std::acos((trace(rot_mat) - 1.0) / 2.0);
        arma::mat eye;
        eye.eye(3, 3);

        arma::vec result =
          (eye - log3 / 2.0 + (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2.0) * log3 * log3 /
          theta) * trans_vec;

        log = {{log3(0, 0), log3(0, 1), log3(0, 2), result[0]},
          {log3(1, 0), log3(1, 1), log3(1, 2), result[1]},
          {log3(2, 0), log3(2, 1), log3(2, 2), result[2]},
          {0.0, 0.0, 0.0, 0.0}};
      }


      // finish logarithm

      arma::Col<double> v_d = {log(2, 1), log(0, 2), log(1, 0), log(0, 3), log(1, 3), log(2, 3)};

      // calculate ff term
      tf_diff = current_config.inverse() * reference_config;

      // make this an armadillo matrix so we can operate easier
      rot = tf_diff.getBasis();
      trans = tf_diff.getOrigin();

      rot_mat = {{rot[0][0], rot[0][1], rot[0][2]},
        {rot[1][0], rot[1][1], rot[1][2]},
        {rot[2][0], rot[2][1], rot[2][2]}};

      arma::Mat<double> so3 = {{0.0, -trans.getZ(), trans.getY()},
        {trans.getZ(), 0.0, -trans.getX()},
        {-trans.getY(), trans.getX(), 0.0}
      };

      arma::Mat<double> dot = so3 * rot_mat;

      arma::Mat<double> adj = {
        {rot_mat(0, 0), rot_mat(0, 1), rot_mat(1, 2), 0.0, 0.0, 0.0},
        {rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2), 0.0, 0.0, 0.0},
        {rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2), 0.0, 0.0, 0.0},
        {dot(0, 0), dot(0, 1), dot(0, 2), rot_mat(0, 0), rot_mat(0, 1), rot_mat(0, 2)},
        {dot(1, 0), dot(1, 1), dot(1, 2), rot_mat(1, 0), rot_mat(1, 1), rot_mat(1, 2)},
        {dot(2, 0), dot(2, 1), dot(2, 2), rot_mat(2, 0), rot_mat(2, 1), rot_mat(2, 2)}
      };

      arma::vec ff_term = adj * v_d;

      arma::vec p_term = x_err * 2.0;

      if (arma::norm(int_x_err + x_err * 1.0 / 50.0) < 0.01) {
        int_x_err = int_x_err + x_err * 1.0 / 50.0;
      }

      arma::vec i_term = int_x_err * 1.0;

      arma::vec control_output = ff_term + p_term + i_term;

      auto cmd_vel = geometry_msgs::msg::Twist();

      cmd_vel.angular.z = control_output[2];
      cmd_vel.linear.x = control_output[3];
      cmd_vel.linear.y = control_output[4];

      pub_cmd_vel_->publish(cmd_vel);
      counter++;
    } else {
      auto cmd_vel = geometry_msgs::msg::Twist();
      pub_cmd_vel_->publish(cmd_vel);
    }

  }

  void odomCallback(const nav_msgs::msg::Odometry & msg)
  {
    odom_msg = msg;
  }

  size_t counter = 0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  nav_msgs::msg::Odometry odom_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path traj;
  arma::vec int_x_err = arma::vec(6, arma::fill::zeros);
  bool received_path = false;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTracker>());
  rclcpp::shutdown();
  return 0;
}
