#ifndef TRAJECTORY_TRANSFORMER_HPP_
#define TRAJECTORY_TRANSFORMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

class TrajectoryTransformer : public rclcpp::Node
{
public:
  TrajectoryTransformer();

private:
  void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr base_link_trajectory_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // TRAJECTORY_TRANSFORMER_HPP_
