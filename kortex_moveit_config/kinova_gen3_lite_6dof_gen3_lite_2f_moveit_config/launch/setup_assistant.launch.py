from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


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
    return generate_setup_assistant_launch(moveit_config)
