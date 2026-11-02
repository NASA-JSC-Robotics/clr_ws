# CLR Connector Mating Demo

A peg-in-hole demo for CLR built on PyTrees behavior trees (`imetro_behavior`), RoboPlan motion planning, and compliant insertion with the stock ros2_controllers admittance controller pair the CLR stack already loads.

The simulated scene adds a connector receptacle with an AprilTag next to the express rack.
A chamfered cylindrical peg is held in the CLR gripper.
The `peg_tip` frame at the end of the peg is the planning and control tip link.

## Running

Start the simulation.

```bash
ros2 launch clr_connector_mating_demo clr_connector_demo_mujoco.launch.py
```

Launch the RoboPlan planning server (planning for `peg_tip`) and the behavior executor:

```bash
ros2 launch clr_connector_mating_demo clr_connector_mating_demo.launch.xml
```

(Optional) Visualize with RViz using the `clr_roboplan_demos` config, and bring up the PyTrees viewer:

```bash
rviz2 -d $(ros2 pkg prefix --share clr_roboplan_demos)/config/clr_roboplan_config.rviz
py-trees-tree-viewer
```

Then run the demo tree:

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: connector_mate}'
```
