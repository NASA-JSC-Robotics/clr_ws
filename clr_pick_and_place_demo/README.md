# CLR CTB Grasping Demonstration

This package contains a demonstration of a perception-enabled CTB picking and placing task.

The application has been used to trial a sim-to-real transfer of simulated behaviors using [MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html).

![alt text](./dynamic_sim_rviz.png "Rviz Rendering of CLR")

## Simulation Instructions

To run this demo in simulation, run the following in a dynamic sim container:

```bash
# Start the mujoco ros2 control-based simulation including the environment
ros2 launch clr_mujoco_config clr_mujoco.launch.py

# Launch the moveit group and the demo's planning GUI
ros2 launch clr_pick_and_place_demo demo_planning_viz.launch.py

# Start the demonstration, the console will display prompts from MoveItVisualTools and provide
# other information.
ros2 launch clr_pick_and_place_demo demo.launch.py
```

By default, each stage of the demonstration requires operator approval by clicking `next` in the RViz console.
To disable this and just let the simulation run, use (still click `next` to start the sim):

```bash
ros2 launch clr_pick_and_place_demo demo.launch.py wait_for_prompt:=false
```

## Hardware Instructions

To run this demo on hardware, run the following in a dynamic sim container on the controls PC:

```bash
ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:="192.168.1.102"

ros2 launch chonkur_deploy ur_tools.launch.py
```

Once the UR communication nodes start, go back over to the console machine and the UR GUI:

```bash
ros2 launch drt_ur_gui one_arm.launch.py
Once the UR status is "READY" use the drop down to select the "load_program" service call and press "Send"

Now that the UR is configured and ready to run, start the CLR drivers.
Use caution!
Do NOT run the robot into anything. 
When indicated, press the safety homing button.

```bash
ros2 launch clr_deploy clr_hw.launch.py include_mockups_in_description:=true
Then run the following in the dynamic sim container on the console PC:

```bash
ros2 launch clr_pick_and_place_demo demo_planning_viz.launch.py sim:=false include_mockups_in_description:=true

ros2 launch clr_pick_and_place_demo demo.launch.py sim:=false
```

Finally, before hitting next, go back to the UR GUI and hit Play.

Then hit next to start the demo.
