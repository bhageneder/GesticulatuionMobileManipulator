#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

class MoveItHandlerNode : public rclcpp::Node {
public:
  MoveItHandlerNode(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_interface,
                    const rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr& wrench_publisher)
    : Node("moveit_handler", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      move_group_interface_(move_group_interface),
      wrench_publisher_(wrench_publisher) {
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

    // Set necessary parameters
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

    // Set the target pose
    move_group_interface_->setPoseTarget(*pose_msg);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful. Executing the plan.");
      // Execute the trajectory
      move_group_interface_->execute(plan);

      // Publish the wrench message
      auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
      joint_state_msg->name = plan.trajectory_.joint_trajectory.joint_names;
      joint_state_msg->position = plan.trajectory_.joint_trajectory.points.back().positions;

      if (joint_state_msg->position.size() >= 6) {
        geometry_msgs::msg::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = this->now(); // Set the timestamp
        wrench_msg.header.frame_id = "base_link"; // Set the frame ID

        wrench_msg.wrench.force.x = joint_state_msg->position[0];
        wrench_msg.wrench.force.y = joint_state_msg->position[1];
        wrench_msg.wrench.force.z = joint_state_msg->position[2];
        wrench_msg.wrench.torque.x = joint_state_msg->position[3];
        wrench_msg.wrench.torque.y = joint_state_msg->position[4];
        wrench_msg.wrench.torque.z = joint_state_msg->position[5];

        wrench_publisher_->publish(wrench_msg);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Not enough joint values in the trajectory for wrench message");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan path");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char * argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<rclcpp::Node>("moveit_handler");

  // Initialize MoveGroupInterface
  auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");

  // Initialize the publisher for the wrench message
  auto wrench_publisher = node->create_publisher<geometry_msgs::msg::WrenchStamped>("joint_positions", 10);

  // Create the MoveItHandlerNode
  auto moveit_handler_node = std::make_shared<MoveItHandlerNode>(move_group_interface, wrench_publisher);

  // Spin the node
  rclcpp::spin(moveit_handler_node);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
