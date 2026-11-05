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
The default, `insertion_smoke_test.yaml`, is a quick two-trial check.
`insertion_offsets.yaml` varies one offset at a time, out to where the insertion fails, and `insertion_lateral_x_tilt_z.yaml` runs every combination of an x offset and a tilt about z.

Results go in a new timestamped folder inside `test_results`, relative to the folder you launched from.
Each trial has a folder with a `bag` folder containing the rosbag, and a `trial.yaml` file containing its parameters and result.
The `trial.yaml` file is written at the end of the trial, so a trial folder without one did not finish.
A trial succeeded if the tree completed and the peg got within 5 mm of the bore floor, and `tree_completed` and `insertion_depth` are also saved.

If an insertion fails, the tree pulls the peg back out before moving on to the next trial.
If that fails as well, the arm is stuck, so the test run ends instead of recording more failed trials.

To plot the results, pass one or more of the timestamped folders to the plotting script:

```bash
python3 src/clr_demos/clr_connector_mating_demo/scripts/plot_insertion_tests.py test_results/<timestamp>
```

This shows how deep the peg got and the peak force for each trial, and saves them in the first folder as `summary.csv` and `summary.png`.
A sweep over two values, such as `insertion_lateral_x_tilt_z.yaml`, is shown as a heatmap in `summary_grid.png` instead.
Depth in the plots is relative to the true receptacle pose, while `insertion_depth` in `trial.yaml` is relative to the AprilTag detection, so the two can differ by a few millimeters.
