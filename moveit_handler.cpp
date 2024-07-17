#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

class MoveItHandlerNode : public rclcpp::Node {
public:
  MoveItHandlerNode(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
                    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_state_publisher)
    : Node("moveit_handler", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      move_group_interface_(move_group_interface),
      joint_state_publisher_(joint_state_publisher) {
    // Subscribe to the geometry_msgs::Pose topic
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "pose_to_moveit", 10, std::bind(&MoveItHandlerNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose_msg) {
    RCLCPP_INFO(this->get_logger(), "Received new target pose");

    // Create a PoseStamped message from the received Pose message
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = this->now();
    pose_stamped_msg.header.frame_id = "world";  // Set the appropriate frame ID
    pose_stamped_msg.pose = *pose_msg;

    //Set necessary parameters
    move_group_interface_->setPlanningTime(0.5); // Set the maximum allowed time for planning
    move_group_interface_->setMaxVelocityScalingFactor(1); // Scale down maximum velocity of the robot
    move_group_interface_->setMaxAccelerationScalingFactor(1); // Scale down maximum acceleration of the robot

    move_group_interface_->setGoalPositionTolerance(0.02); // Position tolerance in meters
    move_group_interface_->setGoalOrientationTolerance(0.05); // Orientation tolerance in radians

    RCLCPP_INFO(this->get_logger(), "info: x position is %f", pose_stamped_msg.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "info: y position is %f", pose_stamped_msg.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "info: z position is %f", pose_stamped_msg.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "info: x orientation is %f", pose_stamped_msg.pose.orientation.x);
    RCLCPP_INFO(this->get_logger(), "info: y orientation is %f", pose_stamped_msg.pose.orientation.y);
    RCLCPP_INFO(this->get_logger(), "INFO FOR ORIENTATION Z IS %f", pose_stamped_msg.pose.orientation.z);

    // Set the target pose based on received pose_msg
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = pose_msg->position.x;
    target_pose.position.y = pose_msg->position.y;
    target_pose.position.z = pose_msg->position.z;
    //target_pose.orientation.x = pose_msg->orientation.x;
    //target_pose.orientation.y = pose_msg->orientation.y;
    //target_pose.orientation.z = pose_msg->orientation.z;
    //target_pose.orientation.w = pose_msg->orientation.w;

    // Set the target pose
    move_group_interface_->setPoseTarget(target_pose);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing the plan.");
      // Execute the trajectory
      move_group_interface_->execute(plan);

      // Publish the joint states
      auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
      joint_state_msg->header.stamp = this->now();
      joint_state_msg->name = plan.trajectory_.joint_trajectory.joint_names;
      joint_state_msg->position = plan.trajectory_.joint_trajectory.points.back().positions;
      joint_state_msg->velocity = plan.trajectory_.joint_trajectory.points.back().velocities;
      joint_state_msg->effort = plan.trajectory_.joint_trajectory.points.back().effort;

      joint_state_publisher_->publish(*joint_state_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan path");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<rclcpp::Node>("moveit_handler");

  // Initialize MoveGroupInterface
  auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");

  // Initialize the publisher for joint states
  auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // Create the MoveItHandlerNode
  auto moveit_handler_node = std::make_shared<MoveItHandlerNode>(move_group_interface, joint_state_publisher);

  // Spin the node
  rclcpp::spin(moveit_handler_node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
