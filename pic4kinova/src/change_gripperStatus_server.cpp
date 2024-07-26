// #include <rclcpp/rclcpp.hpp>
// #include <adriana_interfaces/srv/gripper_status.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// using GripperStatus = adriana_interfaces::srv::GripperStatus;

// class GripperServer : public rclcpp::Node
// {
// public:
//   GripperServer() 
//     : Node("change_gripperStatus_server"), 
//       move_group(std::make_shared<rclcpp::Node>("move_group_node"), "gripper")
//   {
//     move_group.setMaxVelocityScalingFactor(0.1);
//     move_group.setMaxAccelerationScalingFactor(1.0);
//     service_ = this->create_service<GripperStatus>("gripper_status", std::bind(&GripperServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
//   }

// private:
//   void handle_request(const std::shared_ptr<GripperStatus::Request> request,
//                       std::shared_ptr<GripperStatus::Response> response)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received request: %s", request->request.c_str());

//     if (request->request == "opened" || request->request == "closed")
//     {
//       //move_group.setNamedTarget(request->request);
//       move_group.setJointValueTarget("right_finger_bottom_joint", 0.4);
//       //move_group.setGoalPositionTolerance(0.01); //added recently 
//       move_group.setPlanningTime(2.0);

//       bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

//       if (success)
//       {
//         response->response = "gripper " + request->request;
//       }
//       else // AGGGGGIUMGEREEEEEEE cond opened  !!!!!!!!
//       {
//         response->response = "failed to set gripper " + request->request;
//       }
//     }
//     else
//     {
//       response->response = "invalid request";
//     }

//     RCLCPP_INFO(this->get_logger(), "Sending back response: %s", response->response.c_str());
//   }

//   rclcpp::Service<GripperStatus>::SharedPtr service_;
//   moveit::planning_interface::MoveGroupInterface move_group;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<GripperServer>();

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to control the gripper.");

//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <adriana_interfaces/srv/gripper_status.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using GripperStatus = adriana_interfaces::srv::GripperStatus;

class GripperServer : public rclcpp::Node
{
public:
  GripperServer() 
    : Node("change_gripperStatus_server"), 
      move_group(std::make_shared<rclcpp::Node>("move_group_node"), "gripper")
  {
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(1.0);
    service_ = this->create_service<GripperStatus>("gripper_status", std::bind(&GripperServer::handle_request, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_request(const std::shared_ptr<GripperStatus::Request> request,
                      std::shared_ptr<GripperStatus::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request: %s", request->request.c_str());

    if (request->request == "closed")
    {
      //move_group.setNamedTarget(request->request);
      move_group.setJointValueTarget("right_finger_bottom_joint", 0.4);
      //move_group.setGoalPositionTolerance(0.01); //added recently 
      move_group.setPlanningTime(2.0);

      bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        response->response = "gripper " + request->request;
      }
      else 
      {
        response->response = "failed to set gripper " + request->request;
      }
    }
    else if (request->request == "opened")
    {
      move_group.setJointValueTarget("right_finger_bottom_joint", 0.0);
      move_group.setPlanningTime(2.0);

      bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        response->response = "gripper " + request->request;
      }
      else 
      {
        response->response = "failed to set gripper " + request->request;
      }
    }

    else
    {
      response->response = "invalid request";
    }

    RCLCPP_INFO(this->get_logger(), "Sending back response: %s", response->response.c_str());
  }

  rclcpp::Service<GripperStatus>::SharedPtr service_;
  moveit::planning_interface::MoveGroupInterface move_group;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GripperServer>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to control the gripper.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}