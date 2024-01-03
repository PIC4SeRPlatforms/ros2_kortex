from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    xacro_args = {
        "robot_ip": "xxx.yyy.zzz.www",
        "name": "gen3_lite",
        "arm": "gen3_lite",
        "dof": "6",
        "vision": "False",
        "prefix": "",
        "sim_gazebo": "False",
        "sim_ignition": "True",
        "simulation_controllers": "/workspaces/vscode_moveit_humble/src/ros2_kortex/kortex_description/arms/gen3_lite/6dof/config/ros2_controllers.yaml",
        "gripper": "gen3_lite_2f",
    }
    moveit_config = (
        MoveItConfigsBuilder("gen3_lite", package_name="kinova_gen3_lite_6dof_gen3_lite_2f_moveit_config")
        .robot_description(mappings=xacro_args)
        .to_moveit_configs()
    )

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=False))
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

    return ld
