# CLR RoboPlan Demos

This package contains an example of the CLR robot and [RoboPlan](https://github.com/open-planning/roboplan), also utilizing the [RoboPlan ROS wrappers](https://github.com/open-planning/roboplan-ros).

## Interactive Viser Demo

1. Bring up the example.

```bash
ros2 run clr_roboplan_demos viser_ik_rrt.py
```

2. From here, you should move the interactive markers and position the robot at start and goal poses.

    a. When you click "Set Start Pose", a green sphere should appear on the screen at the robot's current location.
    b. When you click "Set Goal Pose", a red sphere should similarly appear on the screen.
    c. Click "Plan Path" to plan from the start to the goal pose.
    d. If planning is successful, click "Animate Trajectory".

**NOTE:** You for now have to manually patch your `ViserVisualizer` with [this bug fix](https://github.com/stack-of-tasks/pinocchio/pull/2878).
To do so, find your `ViserVisualizer` (i.e., through your IDE) and replace its contents with [this version](https://github.com/stack-of-tasks/pinocchio/blob/b432913c28f9a512551fa6dfbaf0eaafee8aa7f9/bindings/python/pinocchio/visualize/viser_visualizer.py).

## MuJoCo Simulation Demo

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
