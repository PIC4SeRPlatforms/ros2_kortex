#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <adriana_interfaces/srv/motion_plan_kinova.hpp>

class MotionPlanServer : public rclcpp::Node
{
public:
    MotionPlanServer()
        : Node("mp_server_adriana")
    {
        service = this->create_service<adriana_interfaces::srv::MotionPlanKinova>(
            "motion_plan_kinova",
            std::bind(&MotionPlanServer::handle_motion_plan_request, this, std::placeholders::_1, std::placeholders::_2));
        movegroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto) {}), "manipulator");
    }

private:
    void handle_motion_plan_request(
        const std::shared_ptr<adriana_interfaces::srv::MotionPlanKinova::Request> request,
        std::shared_ptr<adriana_interfaces::srv::MotionPlanKinova::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received motion plan request");
        geometry_msgs::msg::Pose target_pose = request->mp_pose.pose;
        movegroup->setPoseTarget(target_pose);
        movegroup->setPlanningTime(10.0);
        movegroup->setGoalTolerance(0.1);
        movegroup->setMaxVelocityScalingFactor(0.5);
        movegroup->setMaxAccelerationScalingFactor(0.1);


        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // Updated planning and success check
        auto plan_result = movegroup->plan(my_plan);
        bool success = static_cast<bool>(plan_result);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Motion plan successful, executing movement");
            movegroup->execute(my_plan);
            response->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
            response->success = false;
        }
    }

    rclcpp::Service<adriana_interfaces::srv::MotionPlanKinova>::SharedPtr service;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> movegroup;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}