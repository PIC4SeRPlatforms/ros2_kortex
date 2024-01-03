from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    bringup_pkg = FindPackageShare("kortex_bringup")
    moveit_config_pkg = FindPackageShare("kinova_gen3_lite_6dof_gen3_lite_2f_moveit_config")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_pkg, "launch", "kortex_sim_control.launch.py"])
            ),
            launch_arguments={
                "robot_type": "gen3_lite",
                "gripper": "gen3_lite_2f",
                "dof": "6",
                "name": "gen3_lite",
                "robot_name": "gen3_lite",
                "robot_hand_controller": "gen3_lite_2f_controller",
                "use_sim_time": "true",
                "launch_rviz": "false",
            }.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_config_pkg, "launch", "move_group.launch.py"])),
            launch_arguments={"use_sim_time": "true"}.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit_config_pkg, "launch", "moveit_rviz.launch.py"])),
            launch_arguments={"use_sim_time": "true"}.items(),
        )
    )

    ld.add_action(
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="pic4kinova",
                    executable="home",
                    name="home",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                )
            ],
        )
    )

    return ld
