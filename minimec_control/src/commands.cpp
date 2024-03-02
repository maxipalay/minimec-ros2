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

class Commands : public rclcpp::Node
{
public:
  Commands()
  : Node("commands")
  {
    // parameters declaration
    declare_parameter("rate", 50.0);

    // calculate timer step
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // create publishers and subscribers
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    sub_odom_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&Commands::odomCallback, this, std::placeholders::_1));

    // create main loop timer
    timer_ = create_wall_timer(
      timer_step, std::bind(&Commands::timerCallback, this));

    traj = nav_msgs::msg::Path();
  }

private:
  void timerCallback()
  {
    if (counter == 0) {
      auto initial_transform = tf2::Transform();
      auto final_transform = tf2::Transform();
      final_transform.setOrigin(tf2::Vector3(2.0, 0.0, 0.0));
      auto final_rot = tf2::Quaternion();
      final_rot.setRPY(0.0, 0.0, 3.14159);
      final_transform.setRotation(final_rot);
      int steps = 1000;

      double initial_yaw, initial_pitch, initial_roll;
      tf2::getEulerYPR(
        initial_transform.getRotation().normalized(), initial_yaw, initial_pitch, initial_roll);

      double final_yaw, final_pitch, final_roll;
      tf2::getEulerYPR(
        final_transform.getRotation().normalized(), final_yaw, final_pitch, final_roll);

      auto yaw_diff = final_yaw - initial_yaw;
      auto yaw_increment = yaw_diff / static_cast<double>(steps);

      auto x_diff = final_transform.getOrigin().getX() - initial_transform.getOrigin().getX();
      auto x_increment = x_diff / static_cast<double>(steps);
      for (int i = 0; i < steps; i++) {
        auto pose = tf2::Transform();
        auto rot = tf2::Quaternion();
        rot.setRPY(0.0, 0.0, yaw_increment * static_cast<double>(i));
        pose.setOrigin(tf2::Vector3(x_increment * static_cast<double>(i), 0.0, 0.0));
        pose.setRotation(rot);
        auto msg_tf = geometry_msgs::msg::Transform();
        tf2::toMsg(pose, msg_tf);
        auto msg_pose = geometry_msgs::msg::PoseStamped();
        msg_pose.pose.position.x = msg_tf.translation.x;
        msg_pose.pose.position.y = msg_tf.translation.y;
        msg_pose.pose.position.z = msg_tf.translation.z;
        msg_pose.pose.orientation = msg_tf.rotation;

        traj.poses.insert(
          traj.poses.end(),
          msg_pose);

      }
    }

    if (counter < 999) {

      // current configuration
      auto current_config = tf2::Transform();   // transform odom->body
      tf2::fromMsg(odom_msg.pose.pose, current_config);

      // reference configuration
      auto reference_config = tf2::Transform();
      tf2::fromMsg(traj.poses.at(counter).pose, reference_config);

      // next configuration
      auto next_config = tf2::Transform();
      tf2::fromMsg(traj.poses.at(counter + 1).pose, next_config);

      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "reference_config: " << next_config.getBasis().getRow(0).getY() << std::endl);

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

      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "rotation: " << rot_mat << std::endl);

      arma::vec trans_vec = {trans[0], trans[1], trans[2]};

      RCLCPP_ERROR_STREAM(
        get_logger(), "aaaaa: " << trans_vec << std::endl);

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


      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "rotation: " << rot_mat << std::endl);

      //arma::mat log3 = arma::real(arma::logmat(rot_mat));
      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "reference_config yaw: " << log3 << std::endl);


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


      // AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
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

      //log3 = arma::real(arma::logmat(rot_mat));
// calculate log3

      // arma::mat log3;

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


      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "reference_config yaw: " << log3 << std::endl);


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
      v_d *= 50.0;

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

      arma::vec ff_term = adj * v_d + x_err * 1.0;

      // double twist_z_world = ff_term[2];
      // double twist_x_world = ff_term[3];
      // double twist_y_world = ff_term[4];


      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "ffterm: " << ff_term << std::endl);


      // // calculate rotation matrix elements from odometry

      // auto transform = current_config.inverse(); // transform body->odom;

      // double yaw, pitch, roll;
      // tf2::getEulerYPR(transform.getRotation().normalized(), yaw, pitch, roll);
      // // now we have the yaw in radians, its the only rotation we have (around the z axis)

      // RCLCPP_ERROR_STREAM(
      //     get_logger(), "yaw: " << yaw << std::endl);

      // auto r11 = std::cos(yaw);
      // auto r12 = -std::sin(yaw);
      // auto r21 = std::sin(yaw);
      // auto r22 = std::cos(yaw);
      // auto r33 = 1.0;

      // // calculate body twist
      // auto twist_omega_body = r33 * twist_z_world;
      // auto twist_x_body = r33 * transform.getOrigin().y() * twist_z_world + r11 * twist_x_world + r12 * twist_y_world;
      // auto twist_y_body = -r33 * transform.getOrigin().x() * twist_z_world + r21 * twist_x_world + r22 * twist_y_world;

      auto cmd_vel = geometry_msgs::msg::Twist();
      // cmd_vel.angular.z = twist_omega_body;
      // cmd_vel.linear.x = twist_x_body;
      // cmd_vel.linear.y = twist_y_body;

      cmd_vel.angular.z = ff_term[2];
      cmd_vel.linear.x = ff_term[3];
      cmd_vel.linear.y = ff_term[4];

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

  int counter = 0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  nav_msgs::msg::Odometry odom_msg;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path traj;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Commands>());
  rclcpp::shutdown();
  return 0;
}
