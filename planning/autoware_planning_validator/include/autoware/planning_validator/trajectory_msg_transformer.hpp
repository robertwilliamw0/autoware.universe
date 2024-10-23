// trajectory_msg_transformer.hpp
#ifndef TRAJECTORY_MSG_TRANSFORMER_HPP_
#define TRAJECTORY_MSG_TRANSFORMER_HPP_

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

class MapToBaseLinkTransformer : public Transformer
{
public:
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame,
    const std::string & source_frame,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;
};

// --- transformTrajectory  ---
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

  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::PoseStamped original_pose;
    original_pose.header = trajectory.header;
    original_pose.pose = point.pose;

    geometry_msgs::msg::PoseStamped transformed_pose =
      transformer->transformPose(original_pose, target_frame, source_frame, tf_buffer);

    autoware_planning_msgs::msg::TrajectoryPoint transformed_point;
    transformed_point.pose = transformed_pose.pose;
    transformed_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    transformed_point.lateral_velocity_mps = point.lateral_velocity_mps;
    transformed_point.acceleration_mps2 = point.acceleration_mps2;
    transformed_point.heading_rate_rps = point.heading_rate_rps;
    transformed_point.front_wheel_angle_rad = point.front_wheel_angle_rad;
    transformed_point.rear_wheel_angle_rad = point.rear_wheel_angle_rad;
    transformed_trajectory.points.push_back(transformed_point);
  }

  return transformed_trajectory;
}

class TrajectoryTransformerNode : public rclcpp::Node
{
 
public:
  TrajectoryTransformerNode(const rclcpp::NodeOptions & options);

private:
  void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

  // rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  // rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<Transformer> transformer_;
  std::string target_frame;
  std::string source_frame;
  autoware_planning_msgs::msg::Trajectory original_trajectory_; 
};


}  // namespace planning_validator


#endif  // TRAJECTORY_MSG_TRANSFORMER_HPP_