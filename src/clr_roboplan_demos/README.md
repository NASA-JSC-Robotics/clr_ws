# CLR RoboPlan Demos

This package contains an example of the CLR robot and [RoboPlan](https://github.com/open-planning/roboplan), also utilizing the [RoboPlan ROS wrappers](https://github.com/open-planning/roboplan-ros).

## Instructions

1. Start the MuJoCo simulation. The default flag will include the mockup environment with the robot.

```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```

2. Switch to the integrated controller. (TODO: this should be automated)

```bash
ros2 control switch_controllers --deactivate joint_trajectory_controller lift_position_trajectory_controller rail_position_trajectory_controller --activate clr_joint_trajectory_controller
```

3. Launch the RoboPlan planning and execution node, which also starts up an RViz window with an interactive marker.

```bash
ros2 launch clr_roboplan_demos clr_example_planning.launch.yaml
```
