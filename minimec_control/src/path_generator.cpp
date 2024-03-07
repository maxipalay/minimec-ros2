#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "minimec_msgs/srv/splne_plan_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tinysplinecxx.h"

using namespace std::chrono_literals;

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator()
  : Node("path_generator")
  {

    auto qos_ = rclcpp::SystemDefaultsQoS{};
    qos_.reliable();

    path_service_ =
      create_service<minimec_msgs::srv::SplinePlanRequest>(
      "generate_spline_plan",
      std::bind(&PathGenerator::planCallback, this, std::placeholders::_1, std::placeholders::_2));

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", qos_);

    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("points", qos_);

  }


private:

    void planCallback(const std::shared_ptr<minimec_msgs::srv::SplinePlanRequest::Request> request,
    std::shared_ptr<minimec_msgs::srv::SplinePlanRequest::Response> response){
        // get frame_id from the message
        std::string frame_id = request->frame_id;
        double resolution = request->resolution;
        
        // our generated path
        nav_msgs::msg::Path generated_path = nav_msgs::msg::Path();
        generated_path.header.frame_id = frame_id; 

        if (request->points.size() <= 4){
            RCLCPP_INFO_STREAM(get_logger(), "received number of mpoints should be greater than 4");
            return;
        }
        // instance the spline
        tinyspline::BSpline spline(request->points.size(), 2, 4); // dimension = 2, degree = 4

        // create the control points vector
        std::vector<tsReal> ctrlp = spline.controlPoints();

        // create marker array, the received points will be published here
        auto marker_arr = visualization_msgs::msg::MarkerArray();
        
        // copy the points into array, create markers to publish them
        int n_markers = 0;
        for (size_t i = 0; i < request->points.size(); i++){
            ctrlp[2*i] = static_cast<tsReal>(request->points.at(i).x);
            ctrlp[2*i+1] = static_cast<tsReal>(request->points.at(i).y);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = get_clock()->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = request->points.at(i).x;
            marker.pose.position.y = request->points.at(i).y;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker_arr.markers.insert(
                marker_arr.markers.end(),
                marker);
            n_markers++;
        }
        
        if (msg->heading_mode == "point"){
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = get_clock()->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = request->heading_point.x;
            marker.pose.position.y = request->heading_point.y;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker_arr.markers.insert(
                marker_arr.markers.end(),
                marker);
        }

        markers_publisher_->publish(marker_arr);

        // set the control points
        spline.setControlPoints(ctrlp);
        
        // get the domain min and max values // for our use case they're always [0,1]
        // tsReal domain_min = spline.domain().min();
        tsReal domain_max = spline.domain().max();

        // approximate spline distance (rough estimate)
        tsReal distance = 0.0;
        // we split the line into 100 segments and calculate the 
        // euclidean distance between each pair of points
        for (size_t i = 1; i < 100; i++){
            std::vector<tsReal> result1 = spline.eval(static_cast<double>(i-1)/static_cast<double>(100-1)*static_cast<double>(domain_max)).result();
            std::vector<tsReal> result2 = spline.eval(static_cast<double>(i)/static_cast<double>(100-1)*static_cast<double>(domain_max)).result();
            tinyspline::Vec2 vec1 = tinyspline::Vec2(result1.at(0), result1.at(1));
            tinyspline::Vec2 vec2 = tinyspline::Vec2(result2.at(0), result2.at(1));
            distance += vec1.distance(vec2);
        }

        auto steps = static_cast<size_t>(distance/resolution);

        // get the indices of equally spaced points in the spline
        tinyspline::std_real_vector_out res = spline.equidistantKnotSeq(steps);

        // add points along the generated spline to the path
        for (size_t i = 0; i < steps; i++){
            std::vector<tsReal> result = spline.eval(res.at(i)).result();
            auto pose = geometry_msgs::msg::PoseStamped();
            pose.pose.position.x = static_cast<double>(result.at(0));
            pose.pose.position.y = static_cast<double>(result.at(1));
            auto rot = tf2::Quaternion();
            if (msg->heading_mode == "fixed"){
                rot.setRPY(0.0, 0.0, msg->heading_angle);
            } else if (msg->heading_mode == "point") {
                // get the angle from our current position to the point
                auto angle = std::atan2(msg->heading_point.y-pose.pose.position.y, msg->heading_point.x-pose.pose.position.x);
                rot.setRPY(0.0, 0.0, angle);
            }
            pose.pose.orientation.x = rot.getX();
            pose.pose.orientation.y = rot.getY();
            pose.pose.orientation.z = rot.getZ();
            pose.pose.orientation.w = rot.getW();

            generated_path.poses.insert(
                generated_path.poses.end(),
                pose);
        }

        // publish
        path_publisher_->publish(generated_path);

        // set path in service response
        response->path = generated_path;
    }
    
    rclcpp::Service<minimec_msgs::srv::SplinePlanRequest>::SharedPtr path_service_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathGenerator>());
  rclcpp::shutdown();
  return 0;
}