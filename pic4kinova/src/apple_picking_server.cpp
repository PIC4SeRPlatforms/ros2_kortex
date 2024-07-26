#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "adriana_interfaces/srv/apple_picking.hpp"

class ApplePickingServer : public rclcpp::Node
{
public:
    ApplePickingServer() : Node("apple_picking_server")
    {
        // Crea il servizio per il picking della mela
        service_ = this->create_service<adriana_interfaces::srv::ApplePicking>(
            "apple_picking",
            std::bind(&ApplePickingServer::applePickingCallback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Apple Picking Server is ready.");

        // Crea un nodo per il MoveGroupInterface
        move_group_node_ = std::make_shared<rclcpp::Node>("move_group_interface_node");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(move_group_node_);
        spin_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "manipulator");
    }

    ~ApplePickingServer()
    {
        if (spin_thread_)
        {
            executor_->cancel();
            spin_thread_->join();
        }
    }

private:
    rclcpp::Service<adriana_interfaces::srv::ApplePicking>::SharedPtr service_;
    std::shared_ptr<rclcpp::Node> move_group_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<std::thread> spin_thread_;

    void applePickingCallback(
        const std::shared_ptr<adriana_interfaces::srv::ApplePicking::Request> request,
        std::shared_ptr<adriana_interfaces::srv::ApplePicking::Response> response)
    {
        // Verifica che la posa ricevuta sia nel frame base_link
        if (request->apple_pose.header.frame_id != "base_link")
        {
            RCLCPP_ERROR(this->get_logger(), "Received pose is not in base_link frame");
            response->success = false;
            return;
        }

        // Ottieni la posa corrente del tool nel frame base_link
        geometry_msgs::msg::PoseStamped current_tool_pose = move_group_->getCurrentPose();

        // Verifica che la posa corrente del tool sia effettivamente nel frame base_link
        if (current_tool_pose.header.frame_id != "base_link")
        {
            RCLCPP_ERROR(this->get_logger(), "Current tool pose is not in base_link frame. Found: %s", current_tool_pose.header.frame_id.c_str());
            response->success = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Current tool orientation in base_link frame: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                    current_tool_pose.pose.orientation.x, current_tool_pose.pose.orientation.y, 
                    current_tool_pose.pose.orientation.z, current_tool_pose.pose.orientation.w);

        // Crea la posa target combinando la posizione della mela con l'orientamento corrente del tool
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";  // Esplicitamente impostato a base_link
        target_pose.pose.position = request->apple_pose.pose.position;  // Usa la posizione della mela
        target_pose.pose.orientation = current_tool_pose.pose.orientation;  // Usa l'orientamento corrente del tool nel frame base_link


        // Imposta il target
        move_group_->setPoseTarget(target_pose);
        move_group_->setMaxVelocityScalingFactor(0.1); 
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(5.0);
        move_group_->setGoalTolerance(0.03);

        // Pianifica il movimento
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Motion plan successful. Executing movement.");
            // Esegui il movimento
            move_group_->execute(my_plan);
            response->success = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed.");
            response->success = false;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApplePickingServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}