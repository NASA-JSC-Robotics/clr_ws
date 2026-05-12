ros2 launch clr_mujoco_config clr_mujoco.launch.py

ros2 launch clr_behavior_pick_and_place_demo pick_and_place_behavior.launch.py

ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'BagPickUp'}"

ros2 action send_goal /bt_execution btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'RandomStartTransform'}"