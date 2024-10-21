// trajectory_transformer.cpp
#include "autoware/planning_validator/trajectory_transform.hpp"

TrajectoryTransformer::TrajectoryTransformer() : Node("trajectory_transformer")
{
  // Initialize subscribers and publishers
  trajectory_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/output/trajectory", 10, std::bind(&TrajectoryTransformer::trajectoryCallback, this, std::placeholders::_1));
  base_link_trajectory_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>("/base_link/trajectory", 10);

  // Initialize TF2 listener and buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 
}

void TrajectoryTransformer::trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  autoware_planning_msgs::msg::Trajectory transformed_trajectory;
  transformed_trajectory.header = msg->header; 

  for (auto& point : msg->points) 
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // Get the transform from map to base_link
      transform_stamped = tf_buffer_->lookupTransform(
        "base_link", "map", 
        point.time_from_start,
        rclcpp::Duration::from_seconds(0.1)); 
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      continue; 
    }

    // Transform the pose
    geometry_msgs::msg::PoseStamped transformed_pose;
    transformed_pose.header = msg->header;
    transformed_pose.pose = point.pose; 
    tf2::doTransform(transformed_pose, transformed_pose, transform_stamped);

    // Update the trajectory point
    point.pose = transformed_pose.pose;
    transformed_trajectory.points.push_back(point); 
  }

  base_link_trajectory_pub_->publish(transformed_trajectory);
}
