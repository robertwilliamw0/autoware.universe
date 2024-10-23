// trajectory_msg_transformer.cpp
#include "autoware/planning_validator/trajectory_msg_transformer.hpp" 

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/logging.hpp>


namespace autoware::planning_validator
{

// Implementation of transformPose 
geometry_msgs::msg::PoseStamped MapToBaseLinkTransformer::transformPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame,
  const std::string & source_frame,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(
    target_frame, source_frame,  // Target frame, source frame
    rclcpp::Time(pose.header.stamp), 
    rclcpp::Duration::from_seconds(1.0)); // Timeout
  tf2::doTransform(pose, transformed_pose, transform); 
  return transformed_pose;
}

TrajectoryTransformerNode::TrajectoryTransformerNode(const rclcpp::NodeOptions & options) : Node("trajectory_msg_transformer", options)
{
// public:
//   TrajectoryTransformerNode(const rclcpp::NodeOptions & options)
//   : Node("trajectory_transformer_node", options)
//   {
    // Initialize TF2 buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to the original trajectory topic
    trajectory_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", 10,
      std::bind(&TrajectoryTransformerNode::trajectoryCallback, this, std::placeholders::_1));

    // Publisher for the transformed trajectory
    transformed_trajectory_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "~/output/transformed_trajectory", 10);

    // Get frame IDs from parameters (with defaults)
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("source_frame", "map");
    this->get_parameter("target_frame", target_frame);
    this->get_parameter("source_frame", source_frame);

    // Create an instance of the transformer
    transformer_ = std::make_unique<MapToBaseLinkTransformer>(); 
}

// private:
  void TrajectoryTransformerNode::trajectoryCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    // Store the trajectory message
    original_trajectory_ = *msg; 
    RCLCPP_INFO(this->get_logger(), "trajectoryCallback Initialised");
    // Transform the trajectory using the function from the header
    autoware_planning_msgs::msg::Trajectory transformed_trajectory = 
        transformTrajectory(original_trajectory_, target_frame, source_frame, transformer_, tf_buffer_);

    // Publish the transformed trajectory
    transformed_trajectory_pub_->publish(transformed_trajectory);
  }

  // rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  // rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr transformed_trajectory_pub_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::unique_ptr<Transformer> transformer_;
  // std::string target_frame;
  // std::string source_frame;
  // autoware_planning_msgs::msg::Trajectory original_trajectory_; 
// };

}  // namespace planning_validator


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<autoware::planning_validator::TrajectoryTransformerNode>(options));
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_validator::TrajectoryTransformerNode)