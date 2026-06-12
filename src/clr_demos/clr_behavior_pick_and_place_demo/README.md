# CLR BehaviorTree Pick and Place Demo

This package contains an example of the CLR robot picking up and placing a cargo transfer bag from its spawned location to the top of the bench.

The purpose of this package is demonstration of capabilities using BehaviorTree.cpp and the `drt_behaviors` package.

A comprehensive overview of BT.CPP and its usage is provided in their [documentation](https://www.behaviortree.dev/).

## Instructions

### Setting up

1. Start up the mujoco simulation, the default flag will include the mockup environment with the robot.

```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```

2. Launches nodes for MoveIt's `move_group`, color blob detection, and behavior tree executor.

```bash
ros2 launch clr_behavior_pick_and_place_demo pick_and_place_behavior.launch.py
```

### Running the demo

The default RViz configuration provides a panel for loading, executing, and viewing status from the BehaviorTree.cpp executor.
The list of available trees is available from the drop down menu in the panel.

To execute a bag picking demonstration, run the following trees:

1. `MoveToPerceptionPose`

    This should position the robot so that the wrist camera has the red bag handle in view.

2. `BagPickUp`

    Detects the bag handle, then plans and executes a trajectory to grasp and lift it.

The tree executor also provides an action server for executing trees by name.
The equivalent action calls as the above are:

```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'MoveToPerceptionPose'}"
```

When ready you can execute the `BagPickUp` behavior tree.

```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'BagPickUp'}"
```

### Additional Notes

The setup process spins up a behavior tree executor node responsible for running the demo properly.

The behavior tree executor provides ROS service to list all of the available trees, and a ROS action to execute the registered trees.

To get the list of all available trees:

```bash
ros2 service call /get_loaded_trees btcpp_ros2_interfaces/srv/GetTrees "{}"
```

To execute one of the available trees:

```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: '<name_of_behavior_tree>'}"
```
