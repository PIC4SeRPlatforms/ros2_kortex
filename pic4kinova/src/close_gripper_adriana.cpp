#include <moveit/move_group_interface/move_group_interface.h>

// IT WORKS 
static const rclcpp::Logger LOGGER = rclcpp::get_logger("close_gripper_adriana");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("close_gripper_adriana", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  // per correggere errore plugin 
  
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(1);
  move_group.setJointValueTarget("right_finger_bottom_joint", 0.4);
  //move_group.setNamedTarget("closed");
  move_group.setPlanningTime(4.0);
  move_group.move();


  rclcpp::shutdown();
  return 0;
}