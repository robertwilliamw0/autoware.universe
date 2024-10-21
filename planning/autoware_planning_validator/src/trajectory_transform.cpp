#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class TrajectoryTransformer : public rclcpp::Node
{
public:
  TrajectoryTransformer() : Node("trajectory_transformer")
  {
    // Initialize TF2 listener and buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to the original trajectory topic
    trajectory_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
        "/output/trajectory", 10, 
        std::bind(&TrajectoryTransformer::trajectoryCallback, this, std::placeholders::_1));

    // Publisher for the transformed trajectory
    transformed_trajectory_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
        "/output/trajectory_baselink", 10);
  }

private:
  void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    // Create a new trajectory message to store the transformed data
    autoware_planning_msgs::msg::Trajectory transformed_trajectory;
    transformed_trajectory.header = msg->header; 
    transformed_trajectory.header.frame_id = "base_link"; // Set the new frame ID

    // Loop through each point in the trajectory
    for (auto &point : msg->points) 
    {
      // Transform the pose of the point
      geometry_msgs::msg::PoseStamped transformed_pose;
      geometry_msgs::msg::PoseStamped original_pose;
      original_pose.header = msg->header;
      original_pose.pose = point.pose; 

      try {
        // Get the transform from "map" to "base_link"
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "base_link", "map",  // Target frame, source frame
            rclcpp::Time(msg->header.stamp), 
            rclcpp::Duration::from_seconds(1.0)); // Timeout

        // Apply the transform to the pose
        tf2::doTransform(original_pose, transformed_pose, transform); 

        // Add the transformed pose to the new trajectory point
        autoware_planning_msgs::msg::TrajectoryPoint transformed_point;
        transformed_point.pose = transformed_pose.pose;
        // ... copy other fields from the original point (velocity, acceleration, etc.) ...
        transformed_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
        transformed_point.lateral_velocity_mps = point.lateral_velocity_mps;

        transformed_point.acceleration_mps2 = point.acceleration_mps2;
        transformed_point.heading_rate_rps = point.heading_rate_rps;
        transformed_point.front_wheel_angle_rad = point.front_wheel_angle_rad;
        transformed_point.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
        transformed_trajectory.points.push_back(transformed_point); 

      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
        return; 
      }
    }

    // Publish the transformed trajectory
    transformed_trajectory_pub_->publish(transformed_trajectory);
  }

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTransformer>());
  rclcpp::shutdown();
  return 0;
}