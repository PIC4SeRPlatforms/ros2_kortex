from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

from pic4kinova.utils import (
    generate_moveit_config,
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_servo_launch,
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_sim_time", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("start_sim", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("servo", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("move_group", default_value=True))

    bringup_pkg = FindPackageShare("kortex_bringup")
    moveit_config = generate_moveit_config()

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
            condition=IfCondition(LaunchConfiguration("start_sim")),
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
                    condition=IfCondition(LaunchConfiguration("start_sim")),
                )
            ],
        )
    )

    ld.add_action(
        OpaqueFunction(
            function=generate_move_group_launch,
            args=[ld, moveit_config],
            condition=IfCondition(LaunchConfiguration("move_group")),
        ),
    )

    ld.add_action(
        OpaqueFunction(
            function=generate_moveit_rviz_launch,
            args=[ld, moveit_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    )

    ld.add_action(
        OpaqueFunction(
            function=generate_servo_launch,
            args=[ld, moveit_config],
            condition=IfCondition(LaunchConfiguration("servo")),
        ),
    )

    return ld
