// trajectory_msg_transformer.hpp
// Author: Robert Watkins
// Creation Date: 22/10/2024
#ifndef AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_MSG_TRANSFORMER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_MSG_TRANSFORMER_HPP_

#include "autoware/planning_validator/trajectory_transform_template.hpp"
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

class TrajectoryTransformerNode : public rclcpp::Node
{
 
public:
  TrajectoryTransformerNode(const rclcpp::NodeOptions & options);

private:
  void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<Transformer> transformer_;
  std::string target_frame_;
  std::string source_frame_;
  autoware_planning_msgs::msg::Trajectory original_trajectory_; 
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__TRAJECTORY_MSG_TRANSFORMER_HPP_