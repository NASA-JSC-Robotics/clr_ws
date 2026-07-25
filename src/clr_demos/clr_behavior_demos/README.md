# CLR Python Behavior Demos

This contains PyTrees based behavior demonstrations.

To run examples, first set up the simulation:

```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py

ros2 launch clr_moveit_config clr_moveit.launch.py use_sim_time:=true
```

(Optional but recommended) Bring up the PyTrees viewer:

```bash
py-trees-tree-viewer
```

You can run some examples by specifying trees from the `trees` subfolder of this package.
For example:

```bash
ros2 launch clr_behavior_demos run_behavior.launch.xml gui:=true
```

If `gui:=true`, this will start a GUI for trajectory previewing and stopping.
Set this to false if you don't need to preview any trajectories.

Then, send an action goal with the desired behavior name.

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: pick_and_place_ctb}'
```

## RoboPlan Demos (Experimental)

These demos rely on a RoboPlan planning server (`scripts/roboplan_planning_server.py`).
The server loads its planning scene from the robot description topic and syncs with the `/joint_states` topic.
It offers services that return `trajectory_msgs/JointTrajectory` messages, which are executed directly on the ROS 2 controllers:

- `~/plan_to_joint_state`: free-space planning to a joint configuration
- `~/plan_to_pose`: IK + free-space planning to an end effector pose
- `~/plan_cartesian_path`: straight-line Cartesian motion to a pose
- `~/preview`, `~/execute`, `~/reset`: Trigger services acting on the last
  planned trajectory (preview re-plays the RViz ghost visualization)

By default Cartesian trajectories are timed optimally against the joint limits
(`cartesian_speed_mode: time_optimal`). Set the parameter to `bounded` to cap
the tool speed/acceleration instead — but note that CLR's slow lift joint makes
bounded mode very conservative.

It also serves an interactive marker workflow for testing.

To run, bring up the simulation:

```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```

Then launch the planning server and behavior executor:

```bash
ros2 launch clr_behavior_demos run_roboplan_behavior.launch.xml
```

(Optional) Visualize with RViz using the `clr_roboplan_demos` config:

```bash
rviz2 -d $(ros2 pkg prefix --share clr_roboplan_demos)/config/clr_roboplan_config.rviz
```

Finally, run the demo tree, which free-space plans to a pose, then executes
two straight-line Cartesian motions (out and back):

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: roboplan_tree}'
```

Or, run the full demo:

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: roboplan_pick_and_place_ctb}'
```
