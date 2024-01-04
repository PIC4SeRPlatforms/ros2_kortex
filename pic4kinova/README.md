# PIC4SeR Package for Kinova Gen3 Lite

This package is meant to wrap the original [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex) package making it easier to work with. Moreover it extends the original package by adding some functionalities and correcting some simulation bugs.

## Usage

There is only a [single launch file](./launch/gen3_lite.launch.py) that manages simulation and MoveIt. Launch arguments can be used to configure what should be started every time from the command line.

Example usage to run a simulation with all MoveIt functionalities:
```bash
ros2 launch pic4kinova gen3_lite.launch.py
```

## Simulation

Due to a bug that still needs to be investigate in deep, the gripper moves correctly only after the robot is in the **home** position. A script is added to the launch file to move the robot to this positions after it is spawned. If the simulation starts correctly, the robot should appear in its home position.

There is a timer to ensure that everything is started before running the script, please increase this time if you encounter any problem.


