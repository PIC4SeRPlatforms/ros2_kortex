// ------------- TO DO : delete this node ------------- 

// #include <chrono>
// #include <memory>
// #include <thread>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <moveit_msgs/action/move_group.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// using namespace std::chrono_literals;


// class OLTestClass : public rclcpp::Node
// {
// public:
//     // open loop mode potrebbe essere il flag da inserire quando il controllore ha raggiunto riferimento 
//     OLTestClass() : Node("OL_test_cpp"), open_loop_mode_(true), open_loop_duration_(1s)
//     {
//         // Publishers
//         publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/twist_controller/commands", 10);

//         // Timers
//         timer_ = this->create_wall_timer(50ms, std::bind(&OLTestClass::publish_twist, this));

//         // Inizializza il MoveGroupInterface per il gripper
//         static const std::string PLANNING_GROUP = "gripper";
//         moveit::planning_interface::MoveGroupInterface move_group(close_gripper, PLANNING_GROUP);
//     }

// private:
//     void close_gripper()
//     {
//         RCLCPP_INFO(this->get_logger(), "Closing gripper");
        
//         move_group.setMaxVelocityScalingFactor(1);
//         move_group.setMaxAccelerationScalingFactor(1);

//         move_group.setNamedTarget("closed");
//         move_group.setPlanningTime(10.0);
//         move_group.move();

//         RCLCPP_INFO(this->get_logger(), "Gripper closed successfully");
//     }

//     void publish_twist()
//     {
//         auto msg = geometry_msgs::msg::Twist();

//         if (open_loop_mode_) {
//             if (!open_loop_start_time_) {
//                 open_loop_start_time_ = this->now();
//                 RCLCPP_INFO(this->get_logger(), "Starting open loop motion");
//             }

//             auto current_time = this->now();
//             if ((current_time - *open_loop_start_time_).seconds() < open_loop_duration_.count()) {
//                 msg.linear.z = 0.01; // 1 m/s in direzione Z
//                 RCLCPP_INFO(this->get_logger(), "Executing open loop motion");
//             } else {
//                 // Termina il movimento open loop
//                 open_loop_mode_ = false;
//                 msg.linear.z = 0.0;
//                 RCLCPP_INFO(this->get_logger(), "Open loop motion completed");
//                 // Chiudi il gripper
//                 close_gripper();
//             }
//         }

//         publisher1_->publish(msg);
//     }

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_; 

//     bool open_loop_mode_;
//     std::optional<rclcpp::Time> open_loop_start_time_;
//     std::chrono::duration<double> open_loop_duration_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto OL_test_cpp = std::make_shared<OLTestClass>();
//     rclcpp::spin(OL_test_cpp);
//     rclcpp::shutdown();
//     return 0;
// }