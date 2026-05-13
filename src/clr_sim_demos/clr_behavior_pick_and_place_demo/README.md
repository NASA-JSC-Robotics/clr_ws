# CLR BehaviorTree Pick and Place Demo

This package contains an example of the CLR robot picking up and placing a cargo transfer bag from its spawned location to the top of the bench.

The purpose of this package is demonstrations of capabilities using behavior trees and the `drt_behaviors` package.

For an introduction to behavior tree and their utilization, as well as the core packages that we build off, please refer to https://www.behaviortree.dev/.

## Instructions

### Setting up
1. Start up the mujoco simulation, the default flag will include the mockup environment with the robot.
```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```
2. `pick_and_place_behavior` launch file setups up the MoveIt's `move_group`, color blob detection node, and behavior tree executor.
```bash
ros2 launch clr_behavior_pick_and_place_demo pick_and_place_behavior.launch.py
```

### Running the demo
To successfully pick up the bag you have to position the wrist camera above the cargo transfer bag where it can see the red handle.
You can do that by calling
```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'RandomStartTransform'}"
```
When ready you can execute the `BagPickUp` behavior tree.
```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'BagPickUp'}"
```


### Additional Notes
The setup process spins up a behavior tree executor node responsible for running the demo properly.

The behavior tree executor provides ROS service to list all of the available trees, and a ROS action to execute the registered trees.

To get the list of all available tree:
```bash
ros2 service call /get_loaded_trees btcpp_ros2_interfaces/srv/GetTrees "{}"
```
To execute one of the available trees:
```bash
ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: '<name_of_behavior_tree>'}"
```
