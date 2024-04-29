from moveit_configs_utils import MoveItConfigsBuilder, MoveItConfigs

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_moveit_config(arm="gen3_lite", dof=6, gripper="gen3_lite_2f"):
    controllers = os.path.join(
        get_package_share_directory("kortex_description"),
        "arms",
        arm,
        str(dof) + "dof",
        "config",
        "ros2_controllers.yaml",
    )

    xacro_args = {
        "robot_ip": "xxx.yyy.zzz.www",
        "name": arm,
        "arm": arm,
        "dof": str(dof),
        "vision": "False",
        "prefix": "",
        "sim_gazebo": "False",
        "sim_ignition": "False",
        "simulation_controllers": controllers,
        "gripper": gripper,
    }

    return (
        MoveItConfigsBuilder(arm, package_name=f"kinova_{arm}_{dof}dof_{gripper}_moveit_config")
        .robot_description(mappings=xacro_args)
        .to_moveit_configs()
    )


def get_servo_params(arm="gen3_lite", dof=6, gripper="gen3_lite_2f"):
    param_file_path = os.path.join(
        get_package_share_directory(f"kinova_{arm}_{dof}dof_{gripper}_moveit_config"), "config", "servo.yaml"
    )

    with open(param_file_path, "r") as f:
        yaml_dict = yaml.safe_load(f)

    return {"moveit_servo": yaml_dict}


def generate_move_group_launch(context, ld: LaunchDescription, moveit_config: MoveItConfigs, *args, **kwargs):
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ["DISPLAY"]},
    )


def generate_moveit_rviz_launch(context, ld: LaunchDescription, moveit_config: MoveItConfigs, *args, **kwargs):
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )


def generate_servo_launch(context, ld: LaunchDescription, moveit_config: MoveItConfigs, *args, **kwargs):
    servo_params = get_servo_params()

    yaml.dump(servo_params, open(os.path.expanduser("~/servo.yaml"), "w"))

    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[servo_params, moveit_config.to_dict(), {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        )
    )
