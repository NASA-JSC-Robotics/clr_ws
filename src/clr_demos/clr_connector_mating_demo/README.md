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

## Automated insertion tests

The `connector_mate_tests` tree runs the connector mating tree once per trial of a test file, varying the speed and offset of the insertion.
Each trial is recorded to its own rosbag.

Start the simulation as above, and launch the demo with a dormant rosbag recorder, which the tree controls through its services.

```bash
ros2 launch clr_connector_mating_demo clr_connector_mating_demo.launch.xml record_rosbags:=true
```

Then run the tests:

```bash
ros2 action send_goal /execute_behavior imetro_behavior_msgs/action/ExecuteBehavior '{tree_file_name: connector_mate_tests}'
```

Test files are in `config/insertion_tests`, and you can select one with the `test_file` attribute at the top of `trees/connector_mate_tests.xml`.
The default is a quick two-trial check, and `insertion_offsets.yaml` is the full sweep of 63 trials.

Results go in a new timestamped folder inside `test_results`, relative to the folder you launched from.
Each trial has a folder with a `bag` folder containing the rosbag, and a `trial.yaml` file containing its parameters and whether it succeeded.
The `trial.yaml` file is written at the end of the trial, so a trial folder without one did not finish.
A trial succeeds if the tree completes, which does not on its own mean that the peg was inserted.
