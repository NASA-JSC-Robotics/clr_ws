# CLR Connector Mating Demo

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
ros2 launch clr_connector_mating_demo clr_connector_mating_demo.launch.xml 
```

By default, `gui:=true`, this will start a GUI for trajectory previewing and stopping.
Set this to false if you don't need to preview any trajectories.

Then, send an action goal with the desired behavior name.

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: connector_mating}'