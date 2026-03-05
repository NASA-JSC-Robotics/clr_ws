/**
 *  Copyright (c) 2025, United States Government, as represented by the
 *  Administrator of the National Aeronautics and Space Administration.
 *
 *  All rights reserved.
 *
 *  This software is licensed under the Apache License, Version 2.0
 *  (the "License"); you may not use this file except in compliance with the
 *  License. You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 */

#include <math.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <thread>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <color_tools_msgs/srv/blob_centroid.hpp>

using namespace std::placeholders;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo_exec");

/*
 * Defines a single waypoint in the demonstration and information
 * about how to execute it.
 */
struct Waypoint
{
  geometry_msgs::msg::Pose pose;
  std::vector<double> config;
  std::string planning_group;
  bool plan_cartesian;
  bool is_relative;
  bool is_preset;
  bool use_jconfig;
  std::string preset_name;
  std::string planner = "default";

  /* Constructor for waypoints with joint configuration information. */
  Waypoint(std::vector<double> j_config, std::string group, bool cartesian)
  {
    is_preset = false;
    use_jconfig = true;
    config = j_config;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = false;
  }

  /* Constructor for waypoints with geometry_msgs::msg::Pose. */
  Waypoint(geometry_msgs::msg::Pose wp_pose, std::string group, bool cartesian, bool relative = false)
  {
    is_preset = false;
    use_jconfig = false;
    pose = wp_pose;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = relative;
  }

  /* Constructor for waypoints with pose information. */
  Waypoint(float x, float y, float z, float qx, float qy, float qz, float qw, std::string group, bool cartesian,
           bool relative = false)
  {
    is_preset = false;
    use_jconfig = false;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = relative;
  }

  /* Constructor for preset waypoints, which do not require a pose to be set. */
  Waypoint(std::string name, std::string group)
  {
    is_preset = true;
    use_jconfig = false;
    preset_name = name;
    planning_group = group;
    plan_cartesian = false;
    is_relative = false;
  }
};

/**
 * Helper function to convert a TransformStamped into a PoseStamped.
 */
geometry_msgs::msg::PoseStamped
transform_stamped_to_pose_stamped(const geometry_msgs::msg::TransformStamped& transform_stamped)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = transform_stamped.header;
  pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
  pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
  pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
  pose_stamped.pose.orientation = transform_stamped.transform.rotation;
  return pose_stamped;
}

/**
 * Helper function to apply a rotation about an axis from one PoseStamped to another PoseStamped,
 * while optionally maintaining the rotation. We use this in bench seat opening to incrementally
 * compute Cartesian poses.
 */
geometry_msgs::msg::PoseStamped rotate_about_frame(std::string rotation_frame,
                                                   geometry_msgs::msg::PoseStamped& rotation_frame_pose,
                                                   geometry_msgs::msg::PoseStamped& ee_start_frame_pose,
                                                   double rotation_amount, tf2::Vector3 axis_rotation,
                                                   bool keep_start_orientation)
{
  // Convert poses to tf2::Transform
  tf2::Transform rotation_frame_tf2, ee_start_frame_tf2;
  tf2::fromMsg(rotation_frame_pose.pose, rotation_frame_tf2);
  tf2::fromMsg(ee_start_frame_pose.pose, ee_start_frame_tf2);

  // Get the transform of ee_start_frame w.r.t. the rotation frame
  tf2::Transform rot_frame_to_ee_start_tf2 = rotation_frame_tf2.inverse() * ee_start_frame_tf2;

  // Create rotation transform
  tf2::Quaternion rotation_quat;
  rotation_quat.setRotation(axis_rotation, rotation_amount);
  tf2::Transform rotation_transform(rotation_quat);

  // Apply rotation
  geometry_msgs::msg::TransformStamped result_transform;
  result_transform.header.frame_id = rotation_frame;
  result_transform.transform = tf2::toMsg(rotation_transform * rot_frame_to_ee_start_tf2);

  // convert the result to pose stamped
  geometry_msgs::msg::PoseStamped rotated_pose = transform_stamped_to_pose_stamped(result_transform);

  // orientation stays the same as the input orientation
  if (keep_start_orientation)
  {
    geometry_msgs::msg::Transform start_transform_msg = tf2::toMsg(rot_frame_to_ee_start_tf2);
    rotated_pose.pose.orientation = start_transform_msg.rotation;
  }

  return rotated_pose;
}

/*
 * The RunDemoNode executes a robot demonstration through a series of plan and
 * execute calls to MoveIt's MoveGroupInterface. Demo execution can be triggered
 * by calling the class function run_demo, which is a blocking call.
 */
class RunDemoNode : public rclcpp::Node
{
public:
  RunDemoNode(rclcpp::NodeOptions node_options) : Node("demo_exec", node_options)
  {
    this->parameter_setup();
    color_blob_client = this->create_client<color_tools_msgs::srv::BlobCentroid>("color_blob_find");
    get_planning_scene_client = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_planning_scene_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    if (hw)
    {
      bench_publisher = this->create_publisher<std_msgs::msg::Float64>("bench_position", 10);
    }
    while (!color_blob_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service.");
        return;
      }
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::Duration(1000));
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

  void run_demo()
  {
    this->movegroup_visualtools_setup("ur_manipulator");
    moveit_viz->trigger();
    moveit_viz->prompt("Press next in RvizVisualToolsGUI to start the demo");
    if (!this->demo_home())
    {
      RCLCPP_ERROR(LOGGER, "Failed to reach home pose. Exiting.");
      return;
    }
    if (!this->init())
    {
      RCLCPP_ERROR(LOGGER, "Failed to reach initial pose. Exiting.");
      return;
    }
    if (!this->approach_bench_seat())
    {
      RCLCPP_ERROR(LOGGER, "Failed to approach bench seat. Exiting.");
      return;
    }
    if (!this->open_bench_seat())
    {
      RCLCPP_ERROR(LOGGER, "Failed to open bench seat. Exiting.");
      return;
    }
    if (!this->back_out())
    {
      RCLCPP_ERROR(LOGGER, "Failed to back out from bench seat. Exiting.");
      return;
    }
    if (!this->approach_stow())
    {
      RCLCPP_ERROR(LOGGER, "Failed to approach stow waypoint. Exiting.");
      return;
    }
    if (!this->stow())
    {
      RCLCPP_ERROR(LOGGER, "Failed to stow manipulator. Exiting.");
      return;
    }
    if (!this->traverse_right())
    {
      RCLCPP_ERROR(LOGGER, "Failed to traverse. Exiting.");
      return;
    }
    if (!this->open_grasp())
    {
      RCLCPP_ERROR(LOGGER, "Failed to open grasp. Exiting.");
      return;
    }
    if (!this->approach_ctb_handle())
    {
      RCLCPP_ERROR(LOGGER, "Failed to reach CTB handle. Exiting.");
      return;
    }
    if (!this->close_grasp())
    {
      RCLCPP_ERROR(LOGGER, "Failed to grasp CTB handle. Exiting.");
      return;
    }
    if (!this->lift_ctb())
    {
      RCLCPP_ERROR(LOGGER, "Failed to lift CTB. Exiting.");
      return;
    }
    if (!this->traverse_left_1())
    {
      RCLCPP_ERROR(LOGGER, "Failed to traverse left. Exiting.");
      return;
    }
    if (!this->pre_drop_ctb())
    {
      RCLCPP_ERROR(LOGGER, "Failed to approach CTB dropoff location. Exiting.");
      return;
    }
    if (!this->traverse_left_2())
    {
      RCLCPP_ERROR(LOGGER, "Failed to traverse to CTB dropoff location. Exiting.");
      return;
    }
    if (!this->center_ctb())
    {
      RCLCPP_ERROR(LOGGER, "Failed to center CTB in bench. Exiting.");
      return;
    }
    if (!this->drop_ctb())
    {
      RCLCPP_ERROR(LOGGER, "Failed to lower CTB. Exiting.");
      return;
    }
    if (!this->open_grasp())
    {
      RCLCPP_ERROR(LOGGER, "Failed to release CTB. Exiting.");
      return;
    }
    if (!this->back_off())
    {
      RCLCPP_ERROR(LOGGER, "Failed to back off from CTB. Exiting.");
      return;
    }
    if (!this->stow())
    {
      RCLCPP_ERROR(LOGGER, "Failed to stow manipulator. Exiting.");
      return;
    }
    if (!this->demo_home())
    {
      RCLCPP_ERROR(LOGGER, "Failed to return to home position. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "Demo succeeded!");
  }

  bool demo_home()
  {
    RCLCPP_INFO(LOGGER, "Moving to home position.");
    return plan_and_execute(wp_map.at("demo_home"));
  }

  bool init()
  {
    RCLCPP_INFO(LOGGER, "Moving to initial position.");
    return plan_and_execute(wp_map.at("init"));
  }

  bool approach_bench_seat()
  {
    RCLCPP_INFO(LOGGER, "Approaching bench seat.");
    return plan_and_execute(wp_map.at("approach_bench_seat"));
  }

  bool open_bench_seat()
  {
    RCLCPP_INFO(LOGGER, "Opening bench seat.");
    this->set_collisions(true, "bench_lid", "finger_1_link");
    this->set_collisions(true, "bench_lid", "finger_2_link");
    this->set_collisions(true, "bench_lid", "gripper_base_link");
    // get hinge frame
    geometry_msgs::msg::TransformStamped hinge_in_world;
    if (!this->get_global_transform("bench_lid", hinge_in_world))
    {
      return false;
    }
    auto hinge_pose = transform_stamped_to_pose_stamped(hinge_in_world);
    // get grasp frame
    geometry_msgs::msg::TransformStamped grasp_in_world;
    if (!this->get_global_transform("grasp_frame", grasp_in_world))
    {
      return false;
    }
    auto grasp_pose = transform_stamped_to_pose_stamped(grasp_in_world);
    auto arc_plan = std::vector<Waypoint>();
    for (int i = 1; i <= 8; i++)
    {
      // returns pose in rotation frame
      geometry_msgs::msg::PoseStamped local_pose =
          rotate_about_frame("bench_lid", hinge_pose, grasp_pose, -i * M_PI / 15.6, tf2::Vector3(0, 1, 0), true);
      // cartesian only supports poses, not posestamped; compute in global
      geometry_msgs::msg::Pose global_pose;
      tf2::doTransform(local_pose.pose, global_pose, hinge_in_world);
      arc_plan.push_back(Waypoint(global_pose, "chonkur_grasp", true));
    }
    bool success = this->plan_and_execute(arc_plan);
    this->set_collisions(false, "bench_lid", "finger_1_link");
    this->set_collisions(false, "bench_lid", "finger_2_link");
    this->set_collisions(false, "bench_lid", "gripper_base_link");
    if (success)
    {
      auto bench_open_pos = std_msgs::msg::Float64();
      bench_open_pos.data = 1.57;
      if (hw)
      {
        bench_publisher->publish(bench_open_pos);
      }
    }
    return success;
  }

  bool back_out()
  {
    RCLCPP_INFO(LOGGER, "Backing out from bench seat.");
    return plan_and_execute(wp_map.at("back_out"));
  }

  bool approach_stow()
  {
    RCLCPP_INFO(LOGGER, "Moving rail to approach stow position.");
    return plan_and_execute(wp_map.at("approach_stow"));
  }

  bool stow()
  {
    RCLCPP_INFO(LOGGER, "Stowing manipulator.");
    return plan_and_execute(wp_map.at("stow"));
  }

  bool traverse_right()
  {
    RCLCPP_INFO(LOGGER, "Traversing to CTB location.");
    return plan_and_execute(wp_map.at("traverse_right"));
  }

  bool approach_ctb_handle()
  {
    RCLCPP_INFO(LOGGER, "Detecting CTB handle.");
    // Find red blob in wrist camera image
    auto request = std::make_shared<color_tools_msgs::srv::BlobCentroid::Request>();
    request->color = "red";
    std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Response> response;
    do
    {
      RCLCPP_INFO(LOGGER, "Sending color blob service request.");
      response = this->request_response<rclcpp::Client<color_tools_msgs::srv::BlobCentroid>::SharedPtr,
                                        std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Request>,
                                        std::shared_ptr<color_tools_msgs::srv::BlobCentroid::Response>>(
          color_blob_client, request);
      if (response->centroid_pose.header.frame_id != "")
      {
        RCLCPP_INFO(LOGGER, "Blob frame id: %s", response->centroid_pose.header.frame_id.c_str());
        break;
      }
      else if (response->color_img.header.frame_id != "")
      {
        RCLCPP_ERROR(LOGGER,
                     "Failed to find %s color blob in FOV. Move robot to "
                     "provide new view.",
                     request->color.c_str());
        return false;
      }
      else
      {
        RCLCPP_WARN(LOGGER, "No image in color blob response. Check that image "
                            "topics exist and data is flowing.");
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } while (response->color_img.header.frame_id == "");

    // Transform waypoint from camera frame to planning frame
    geometry_msgs::msg::Pose local_pose = response->centroid_pose.pose;
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::Pose global_pose;
    bool success = this->get_global_transform(response->centroid_pose.header.frame_id, transform);
    if (!success)
    {
      return false;
    }
    tf2::doTransform(local_pose, global_pose, transform);

    // Add offset to place grasp frame below the CTB handle
    geometry_msgs::msg::Pose offset = wp_map.at("ctb_offset").pose;
    global_pose = this->relative_to_global(global_pose, offset);

    // TODO: Consider replacing with a two phase grasp, one to align to fixed z-offset, and one to grasp.
    RCLCPP_INFO(LOGGER, "Reaching for CTB handle.");
    Waypoint blob_wp = Waypoint(global_pose, "chonkur_grasp", true);
    return plan_and_execute(blob_wp);
  }

  bool close_grasp()
  {
    RCLCPP_INFO(LOGGER, "Closing grasp.");
    return plan_and_execute(wp_map.at("close_grasp"));
  }

  bool lift_ctb()
  {
    RCLCPP_INFO(LOGGER, "Lifting CTB.");
    return plan_and_execute(wp_map.at("lift_relative")) && plan_and_execute(wp_map.at("lift_lift")) &&
           plan_and_execute(wp_map.at("stow_ctb"));
  }

  bool traverse_left_1()
  {
    RCLCPP_INFO(LOGGER, "Traversing to bench seat location.");
    return plan_and_execute(wp_map.at("traverse_left_1"));
  }

  bool pre_drop_ctb()
  {
    RCLCPP_INFO(LOGGER, "Approaching CTB dropoff location.");
    geometry_msgs::msg::TransformStamped eef;
    if (!this->get_global_transform("tool0", eef))
    {
      return false;
    }
    Waypoint approach_wp_1 = Waypoint(eef.transform.translation.x, eef.transform.translation.y,
                                      eef.transform.translation.z, 0.725, 0.688, 0.032, -0.004, "ur_manipulator", true);
    // return plan_and_execute({ approach_wp_1, wp_map.at("pre_drop_ctb") });
    return plan_and_execute(approach_wp_1) && plan_and_execute(wp_map.at("pre_drop_ctb"));
  }

  bool traverse_left_2()
  {
    RCLCPP_INFO(LOGGER, "Traversing to bench seat location.");
    return plan_and_execute(wp_map.at("traverse_left_2"));
  }

  bool center_ctb()
  {
    RCLCPP_INFO(LOGGER, "Centering CTB in bench.");
    return plan_and_execute(wp_map.at("center_ctb"));
  }

  bool drop_ctb()
  {
    RCLCPP_INFO(LOGGER, "Dropping CTB.");
    if (!plan_and_execute(wp_map.at("drop_arm")))
    {
      return false;
    }
    return plan_and_execute(wp_map.at("drop_lift"));
  }

  bool open_grasp()
  {
    RCLCPP_INFO(LOGGER, "Closing grasp.");
    return plan_and_execute(wp_map.at("open_grasp"));
  }

  bool back_off()
  {
    RCLCPP_INFO(LOGGER, "Backing off.");
    return plan_and_execute(wp_map.at("back_off"));
  }

  bool plan_and_execute(const Waypoint& waypoint, bool setup = true)
  {
    if (setup)
    {
      this->movegroup_visualtools_setup(waypoint.planning_group);
    }
    if (waypoint.planner != "default")
    {
      move_group->setPlannerId(waypoint.planner);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = false;
    int attempts = 0;

    while (!success && attempts < 4)
    {
      if (waypoint.is_preset || !waypoint.plan_cartesian)
      {
        success = this->plan_to_pose(waypoint, trajectory);
      }
      else
      {
        success = this->plan_cartesian(waypoint, trajectory);
      }
      attempts += 1;
    }

    if (!success)
    {
      return false;
    }
    else
    {
      if (wait_for_prompt)
      {
        this->prompt(trajectory);
      }
      success = this->execute_trajectory(trajectory);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return success;
  }

  bool plan_and_execute(const std::vector<Waypoint>& waypoints, bool setup = true)
  {
    std::string group = waypoints[0].planning_group;
    std::vector<geometry_msgs::msg::Pose> poses;
    for (const auto& waypoint : waypoints)
    {
      // Waypoints must share a planning group. Relative waypoints not yet
      // supported. Cartesian planning does not support joint configurations or
      // presets.
      if (waypoint.planning_group != group || waypoint.is_relative || waypoint.use_jconfig || waypoint.is_preset)
      {
        RCLCPP_WARN(LOGGER, "Cannot plan a Cartesian trajectory for provided waypoints.");
        return false;
      }
      else if (!waypoint.plan_cartesian)
      {
        RCLCPP_WARN(LOGGER, "Cannot plan non-Cartesian trajectory for multiple waypoints.");
        return false;
      }
      poses.push_back(waypoint.pose);
    }
    if (setup)
    {
      this->movegroup_visualtools_setup(group);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = this->plan_cartesian(poses, trajectory);

    if (!success)
    {
      return false;
    }
    else
    {
      if (wait_for_prompt)
      {
        this->prompt(trajectory);
      }
      success = this->execute_trajectory(trajectory);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return success;
  }

  // Whether to prompt operator before executing a trajectory. ROS parameter.
  bool wait_for_prompt;
  // Factor (<=1.0) by which to scale velocity and acceleration. ROS parameter.
  float scaling;
  // If true, the bench seat position will be published as a substitute for
  // Mujoco joint states. ROS parameter.
  bool hw;
  // Named Waypoints loaded from a yaml file specified by the waypoint_cfg ROS
  // parameter.
  std::map<std::string, Waypoint> wp_map;
  // MoveIt move_group object. A new move_group object must be created for each
  // planning group.
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  // moveit_visual_tools object. A new moveit_visual_tools object must be
  // created for each planning group.
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_viz;
  // Service client for color_blob_find, which identifies the centroid of a
  // region of a specified color.
  rclcpp::Client<color_tools_msgs::srv::BlobCentroid>::SharedPtr color_blob_client;
  // GetPlanningScene service client to access Allowed Collision Matrix (ACM).
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_client;
  // ApplyPlanningScene service client to modify Allowed Collision Matrix (ACM).
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client;
  // Publishes position of bench seat when mockup joint states are not available
  // (i.e. hardware).
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bench_publisher;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

private:
  // Set collisions for a link or pair of links. If collisions are allowed,
  // collision checking is disabled and collisions can occur. Setting collisions
  // for a link without specifying a second link enables collisions with all
  // other links.
  bool set_collisions(bool allowed, std::string link1, std::string link2 = "")
  {
    // Get planning scene
    RCLCPP_INFO(LOGGER, "Requesting planning scene.");
    auto get_planning_scene = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = this->request_response<rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr,
                                           std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>,
                                           std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response>>(
        get_planning_scene_client, get_planning_scene);
    // Modify ACM
    auto acm = collision_detection::AllowedCollisionMatrix(response->scene.allowed_collision_matrix);
    if (link2 == "")
    {
      acm.setEntry(link1, allowed);
    }
    else
    {
      acm.setEntry(link1, link2, allowed);
    }
    auto apply_planning_scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
    acm.getMessage(acm_msg);
    apply_planning_scene->scene.allowed_collision_matrix = acm_msg;
    apply_planning_scene->scene.is_diff = true;
    // Apply planning scene
    RCLCPP_INFO(LOGGER, "Applying planning scene.");
    auto apply_response = this->request_response<rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr,
                                                 std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request>,
                                                 std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response>>(
        apply_planning_scene_client, apply_planning_scene);
    if (!apply_response->success)
    {
      if (link2 == "")
      {
        RCLCPP_ERROR(LOGGER, "Failed to disable collisions with %s.", link1.c_str());
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Failed to disable collisions between %s and %s.", link1.c_str(), link2.c_str());
      }
      return false;
    }
    return true;
  }

  // Generic synchronous service call.
  template <typename Client, typename Request, typename Response>
  Response request_response(Client client, Request request)
  {
    auto future = client->async_send_request(request);
    // Wait for the result.
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
      RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 1000, "Waiting for service response...");
    }
    return future.get();
  }

  // Look up transform between frame_id and the world frame.
  bool get_global_transform(const std::string& frame_id, geometry_msgs::msg::TransformStamped& t)
  {
    try
    {
      t = this->tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero, std::chrono::nanoseconds(5000));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_INFO(this->get_logger(), "Could not get transform from %s to world: %s", frame_id.c_str(), ex.what());
      return false;
    }
    return true;
  }

  // Return end_pose transformed from the end-effector frame to the global
  // frame. start_pose must be the pose of the end-effector frame in the global
  // frame.
  geometry_msgs::msg::Pose relative_to_global(const geometry_msgs::msg::Pose& start_pose,
                                              const geometry_msgs::msg::Pose& end_pose)
  {
    auto pose_action_eigen = moveit_visual_tools::MoveItVisualTools::convertPose(end_pose);
    auto start_pose_eigen = moveit_visual_tools::MoveItVisualTools::convertPose(start_pose);
    auto end_pose_eigen = start_pose_eigen * pose_action_eigen;
    return moveit_visual_tools::MoveItVisualTools::convertPose(end_pose_eigen);
  }

  // Generate a Cartesian plan for a single Waypoint. Handles relative
  // Waypoints.
  bool plan_cartesian(const Waypoint& waypoint, moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
    geometry_msgs::msg::Pose end_pose = waypoint.pose;
    if (waypoint.is_relative)
    {
      end_pose = this->relative_to_global(start_pose, end_pose);
    }
    std::vector<geometry_msgs::msg::Pose> poses = { end_pose };
    return this->plan_cartesian(poses, trajectory);
  }

  // Generate a Cartesian plan to one or more poses. Includes velocity and
  // acceleration scaling. Waypoints must share a planning group and cannot be
  // relative poses, joint configs, or presets.
  bool plan_cartesian(const std::vector<geometry_msgs::msg::Pose>& poses, moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Using planning group: %s", move_group->getName().c_str());

    const double eef_step = 0.01;

    double trajectory_percent = move_group->computeCartesianPath(poses, eef_step, trajectory);

    if (trajectory_percent == 1.0)
    {
      RCLCPP_INFO(LOGGER, "Successfully computed 100%% of trajectory");
    }
    else
    {
      RCLCPP_ERROR(LOGGER,
                   "Error - Cartesian path plan (%.2f%% achieved). Cannot "
                   "execute trajectory.",
                   trajectory_percent * 100.0);
      return false;
    }

    robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), move_group->getName());
    rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

    // Cannot do velocity and acceleration scaling with Cartesian planning, as
    // described below:
    // https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html
    // The page below is referenced, which recommends manual velocity scaling:
    // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;

    bool success;
    success = totg.computeTimeStamps(rt, scaling, scaling);
    RCLCPP_INFO(LOGGER, "Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");

    if (success)
    {
      // Get RobotTrajectory_msg from RobotTrajectory
      rt.getRobotTrajectoryMsg(trajectory);
      RCLCPP_INFO(LOGGER, "Successfully scaled Cartesian trajectory");
      return true;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to scale Cartesian trajectory");
      return false;
    }
  }

  // Generate motion plan to Waypoint using its specified planning group and
  // planner. Supports relative poses, presets, and joint configs.
  bool plan_to_pose(const Waypoint& waypoint, moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Using planning group: %s", move_group->getName().c_str());

    if (waypoint.is_preset)
    {
      move_group->setJointValueTarget(move_group->getNamedTargetValues(waypoint.preset_name));
    }
    else if (waypoint.use_jconfig)
    {
      move_group->setJointValueTarget(waypoint.config);
    }
    else
    {
      geometry_msgs::msg::Pose end_pose = waypoint.pose;
      if (waypoint.is_relative)
      {
        geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
        end_pose = this->relative_to_global(start_pose, end_pose);
      }
      move_group->setPoseTarget(end_pose);
    }

    move_group->setNumPlanningAttempts(5);
    move_group->setMaxVelocityScalingFactor(scaling);
    move_group->setMaxAccelerationScalingFactor(scaling);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error_code = move_group->plan(plan);

    if (error_code)
    {
      RCLCPP_INFO(LOGGER, "Successfully computed trajectory");
      trajectory = plan.trajectory;
      return true;
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Planning failed: %s", error_code.message.c_str());
      return false;
    }
  }

  bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    moveit::core::MoveItErrorCode move_success = move_group->execute(trajectory);
    if (move_success != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Execution failed with error code %s", move_success.message.c_str());
      return false;
    }
    return true;
  }

  // Use moveit_visual_tools to request operator approval of plan.
  void prompt(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    // moveit_visual_tools fails to update state of rail and lift if they are
    // not in the current planning group.
    if (move_group->getName() == "clr")
    {
      moveit_viz->publishTrajectoryLine(trajectory,
                                        move_group->getRobotModel()->getJointModelGroup(move_group->getName()));
    }
    moveit_viz->trigger();
    moveit_viz->prompt("Press next to perform motion");
  }

  // Set up move_group and moveit_visual_tools based on the current planning
  // group.
  void movegroup_visualtools_setup(const std::string& planning_group)
  {
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), planning_group);

    moveit_viz =
        std::make_unique<moveit_visual_tools::MoveItVisualTools>(shared_from_this(), move_group->getPlanningFrame(),
                                                                 rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                 move_group->getRobotModel());

    moveit_viz->loadMarkerPub();
    moveit_viz->enableBatchPublishing();
    moveit_viz->deleteAllMarkers();
    moveit_viz->loadRemoteControl();
  }

  // Set up ROS parameters.
  void parameter_setup()
  {
    auto prompt_desc = rcl_interfaces::msg::ParameterDescriptor{};
    prompt_desc.description = "Whether to prompt before executing a trajectory.";
    auto scaling_desc = rcl_interfaces::msg::ParameterDescriptor{};
    prompt_desc.description = "Factor (<=1.0) by which to scale velocity and acceleration.";
    auto hw_desc = rcl_interfaces::msg::ParameterDescriptor{};
    prompt_desc.description = "If true, the bench seat position will be published as a substitute "
                              "for Mujoco joint states.";

    if (!this->has_parameter("wait_for_prompt"))
    {
      this->declare_parameter("wait_for_prompt", true, prompt_desc);
    }
    if (!this->has_parameter("scaling_factor"))
    {
      this->declare_parameter("scaling_factor", 1.0, scaling_desc);
    }
    if (this->has_parameter("waypoint_cfg"))
    {
      load_waypoints_from_yaml(this->get_parameter("waypoint_cfg").as_string());
    }
    if (!this->has_parameter("hw"))
    {
      this->declare_parameter("hw", false, prompt_desc);
    }
    wait_for_prompt = this->get_parameter("wait_for_prompt").as_bool();
    scaling = this->get_parameter("scaling_factor").as_double();
    hw = this->get_parameter("hw").as_bool();
  }

  // Load Waypoints from yaml file. Waypoints must include all required fields.
  void load_waypoints_from_yaml(std::string demo_config)
  {
    auto wp_yaml = YAML::LoadFile(demo_config);
    if (!wp_yaml["waypoints"])
    {
      RCLCPP_WARN(LOGGER, "Error loading YAML test/config");
    }
    for (YAML::const_iterator it = wp_yaml["waypoints"].begin(); it != wp_yaml["waypoints"].end(); ++it)
    {
      if (it->second["config"])
      {
        Waypoint loaded_waypoint =
            Waypoint(it->second["config"].as<std::vector<double>>(), it->second["planning_group"].as<std::string>(),
                     it->second["cartesian"].as<bool>());
        if (it->second["planner"])
          loaded_waypoint.planner = it->second["planner"].as<std::string>();
        wp_map.insert({ it->first.as<std::string>(), loaded_waypoint });
      }
      else if (it->second["pose"])
      {
        Waypoint loaded_waypoint =
            Waypoint(it->second["pose"]["x"].as<double>(), it->second["pose"]["y"].as<double>(),
                     it->second["pose"]["z"].as<double>(), it->second["pose"]["qx"].as<double>(),
                     it->second["pose"]["qy"].as<double>(), it->second["pose"]["qz"].as<double>(),
                     it->second["pose"]["qw"].as<double>(), it->second["planning_group"].as<std::string>(),
                     it->second["cartesian"].as<bool>());
        if (it->second["relative"])
          loaded_waypoint.is_relative = it->second["relative"].as<bool>();
        if (it->second["planner"])
          loaded_waypoint.planner = it->second["planner"].as<std::string>();
        wp_map.insert({ it->first.as<std::string>(), loaded_waypoint });
      }
      else if (it->second["preset_name"])
      {
        Waypoint loaded_waypoint =
            Waypoint(it->second["preset_name"].as<std::string>(), it->second["planning_group"].as<std::string>());
        if (it->second["planner"])
          loaded_waypoint.planner = it->second["planner"].as<std::string>();
        wp_map.insert({ it->first.as<std::string>(), loaded_waypoint });
      }
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Construct the demo node
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto run_demo_node = std::make_shared<RunDemoNode>(node_options);

  // Start the executor spinning
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(run_demo_node);

  // Run the demo in the background
  std::thread demo_thread(&RunDemoNode::run_demo, run_demo_node);

  // Spin until terminated
  executor.spin();

  run_demo_node.reset();
  rclcpp::shutdown();

  return 0;
}
