/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef CLR_MUJOCO_PLUGINS__JOINT_LIMIT_MONITOR_PLUGIN_HPP_
#define CLR_MUJOCO_PLUGINS__JOINT_LIMIT_MONITOR_PLUGIN_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace clr_mujoco_plugins {

/**
 * @brief Tracks whether a single limit is being violated.
 */
struct LimitState {
  // Sim time at which the current violation started. Empty while within limits.
  std::optional<double> violation_start;
  // Once latched, the limit stays violated forever.
  bool latched{false};
};

/**
 * @brief A monitored hinge or slide joint and its configured limits.
 */
struct MonitoredJoint {
  std::string name;
  int qpos_adr{-1};
  int dof_adr{-1};

  std::optional<double> min_position;
  std::optional<double> max_position;
  std::optional<double> max_velocity;
  std::optional<double> max_acceleration;

  LimitState min_position_state;
  LimitState max_position_state;
  LimitState max_velocity_state;
  LimitState max_acceleration_state;
};

/**
 * @brief Monitors joints for position, velocity, and acceleration limit
 * violations.
 *
 * A limit is considered violated once it has been exceeded continuously for
 * `violation_duration` seconds of simulation time. Violations latch: the plugin
 * publishes a transient-local std_msgs/String listing every violated joint and
 * limit, which is republished whenever a new violation is added and is never
 * cleared.
 *
 * Example configuration:
 *
 *   joint_limit_monitor:
 *     type: "clr_mujoco_plugins/JointLimitMonitorPlugin"
 *     topic: "joint_limit_violations"
 *     violation_duration: 0.5
 *     joints:
 *       shoulder_pan_joint:
 *         min_position: -3.14
 *         max_position: 3.14
 *         max_velocity: 2.0
 *         max_acceleration: 10.0
 *
 * Every limit is optional. Joints that are not in the MuJoCo model are skipped
 * with a warning.
 */
class JointLimitMonitorPlugin
    : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
  JointLimitMonitorPlugin() = default;
  ~JointLimitMonitorPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel *model,
            mjData *data) override;

  void update(const mjModel *model, mjData *data) override;

  void cleanup() override;

  /**
   * @brief Getter for the monitored joints, used for testing
   */
  const std::vector<MonitoredJoint> &get_monitored_joints() const {
    return joints_;
  }

  /**
   * @brief Getter for the latched violation report (empty if nothing has been
   * violated), used for testing
   */
  std::string get_violation_report() const;

private:
  /**
   * @brief Parses the joints parameters and registers those that exist in the
   * model.
   */
  bool register_joints(const mjModel *model);

  /**
   * @brief Updates the state of one limit, latching it if it has been violated
   * for long enough.
   * @return true if the limit was newly latched
   */
  bool check_limit(const std::string &joint_name, const std::string &limit_name,
                   double limit, double value, bool is_violated, double time,
                   LimitState &state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("JointLimitMonitorPlugin");
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr violation_publisher_;

  std::vector<MonitoredJoint> joints_;
  double violation_duration_{0.5};
  // Used to detect world resets, which rewind the sim clock.
  double last_time_{0.0};
  // One line per latched violation, in the order they latched.
  std::vector<std::string> violations_;
};

} // namespace clr_mujoco_plugins

#endif // CLR_MUJOCO_PLUGINS__JOINT_LIMIT_MONITOR_PLUGIN_HPP_
