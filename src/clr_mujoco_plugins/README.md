# clr_mujoco_plugins

CLR-specific [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) plugins.

## JointLimitMonitorPlugin

Monitors a set of joints for position, velocity, and acceleration limit violations.
A limit counts as violated once it has been exceeded continuously for `violation_duration` seconds of simulation time.
Violations latch: once a joint violates a limit it stays reported, even after it returns within limits or the world is reset.

Every time a new violation latches, the plugin publishes a `std_msgs/String` listing all violated joints and limits on a transient-local topic, so late subscribers still receive it.

```yaml
/**:
  ros__parameters:
    mujoco_plugins:
      joint_limit_monitor:
        type: "clr_mujoco_plugins/JointLimitMonitorPlugin"
        # Relative to the plugin namespace, so this resolves to /joint_limit_monitor/joint_limit_violations
        topic: "joint_limit_violations"
        violation_duration: 0.5
        joints:
          shoulder_pan_joint:
            min_position: -3.14
            max_position: 3.14
            max_velocity: 2.0        # applied to |qvel|
            max_acceleration: 10.0   # applied to |qacc|
```

All limits are optional.
Joints that are not in the MuJoCo model, or are not hinge/slide joints, are skipped with a warning.
Units are radians for hinge joints and meters for slide joints.

```bash
ros2 topic echo --qos-durability transient_local --qos-reliability reliable /joint_limit_monitor/joint_limit_violations
```
