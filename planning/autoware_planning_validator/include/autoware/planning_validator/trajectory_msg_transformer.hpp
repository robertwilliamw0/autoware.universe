// trajectory_msg_transformer.hpp
#ifndef TRAJECTORY_MSG_TRANSFORMER_HPP_
#define TRAJECTORY_MSG_TRANSFORMER_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>


namespace autoware::planning_validator
{

class TrajectoryTransformer : public rclcpp::Node
{
  // explicit PlanningValidator(const rclcpp::NodeOptions & options);
public:
  TrajectoryTransformer(const rclcpp::NodeOptions & options);

private:
  void trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace planning_validator


#endif  // TRAJECTORY_MSG_TRANSFORMER_HPP_