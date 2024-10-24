// trajectory_transform_template.hpp
// Author: Robert Watkins
// Creation Date: 22/10/2024
#ifndef AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_TRANSFORM_TEMPLATE_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_TRANSFORM_TEMPLATE_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

namespace autoware::planning_validator
{

class Transformer
{
public:
  virtual geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame,
    const std::string & source_frame,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) = 0;
};

//Inherited in the chance we want to always transform between map or baselink. Future proofing.
class MapToBaseLinkTransformer : public Transformer 
{
public:
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame,
    const std::string & source_frame,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;
};

// --- Transform Trajectory msg ---  //
autoware_planning_msgs::msg::Trajectory transformTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::string & target_frame,
  const std::string & source_frame,
  const std::unique_ptr<Transformer> & transformer,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  autoware_planning_msgs::msg::Trajectory transformed_trajectory;
  transformed_trajectory.header = trajectory.header;
  transformed_trajectory.header.frame_id = target_frame;

  // Loop through the array
  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::PoseStamped original_pose;
    original_pose.header = trajectory.header;
    original_pose.pose = point.pose;

    geometry_msgs::msg::PoseStamped transformed_pose =
      transformer->transformPose(original_pose, target_frame, source_frame, tf_buffer);

    autoware_planning_msgs::msg::TrajectoryPoint transformed_point;
    transformed_point.pose = transformed_pose.pose;
    // Populating the rest of the point message with the relevant information
    // Possible to do any additional conversions needed. Assumed not needed as variables are local/global independent.
    transformed_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    transformed_point.lateral_velocity_mps = point.lateral_velocity_mps;
    transformed_point.acceleration_mps2 = point.acceleration_mps2;
    transformed_point.heading_rate_rps = point.heading_rate_rps;
    transformed_point.front_wheel_angle_rad = point.front_wheel_angle_rad;
    transformed_point.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
    transformed_trajectory.points.push_back(transformed_point); //adding new point to the array
  }

  return transformed_trajectory;
}

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_TRANSFORM_TEMPLATE_HPP_