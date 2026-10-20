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

#include "joint_limit_monitor_plugin.hpp"

#include <cmath>
#include <set>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

namespace clr_mujoco_plugins {

namespace {

/// Reads an optional number parameter. Integers are accepted so that e.g.
/// `max_velocity: 2` works.
std::optional<double> get_optional_double(const rclcpp::Node::SharedPtr &node,
                                          const std::string &name) {
  if (!node->has_parameter(name)) {
    return std::nullopt;
  }
  const auto param = node->get_parameter(name);
  if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(param.as_int());
  }
  // Throws if the parameter is not a double.
  return param.as_double();
}

} // namespace

bool JointLimitMonitorPlugin::register_joints(const mjModel *model) {
  const std::string joints_namespace =
      "mujoco_plugins." + node_->get_sub_namespace() + ".joints";
  const std::string joints_prefix = joints_namespace + ".";

  // Collect the unique joint names under mujoco_plugins.<plugin_name>.joints.
  const auto param_names =
      node_
          ->list_parameters(
              {joints_namespace},
              rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE)
          .names;
  std::set<std::string> joint_names;
  for (const auto &name : param_names) {
    if (name.find(joints_prefix) != 0) {
      continue;
    }
    const std::string relative_path = name.substr(joints_prefix.length());
    const size_t dot_pos = relative_path.find('.');
    if (dot_pos != std::string::npos) {
      joint_names.insert(relative_path.substr(0, dot_pos));
    }
  }

  joints_.clear();
  for (const auto &joint_name : joint_names) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id == -1) {
      RCLCPP_WARN(
          logger_,
          "Joint '%s' not found in the model, it will not be monitored.",
          joint_name.c_str());
      continue;
    }
    const int joint_type = model->jnt_type[joint_id];
    if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
      RCLCPP_WARN(
          logger_,
          "Joint '%s' is not a hinge or slide joint, it will not be monitored.",
          joint_name.c_str());
      continue;
    }

    MonitoredJoint joint;
    joint.name = joint_name;
    joint.qpos_adr = model->jnt_qposadr[joint_id];
    joint.dof_adr = model->jnt_dofadr[joint_id];

    const std::string joint_param_prefix = joints_prefix + joint_name + ".";
    joint.min_position =
        get_optional_double(node_, joint_param_prefix + "min_position");
    joint.max_position =
        get_optional_double(node_, joint_param_prefix + "max_position");
    joint.max_velocity =
        get_optional_double(node_, joint_param_prefix + "max_velocity");
    joint.max_acceleration =
        get_optional_double(node_, joint_param_prefix + "max_acceleration");

    if (joint.min_position && joint.max_position &&
        *joint.min_position > *joint.max_position) {
      RCLCPP_ERROR(logger_,
                   "Joint '%s' has min_position greater than max_position.",
                   joint_name.c_str());
      return false;
    }

    RCLCPP_INFO(logger_, "Monitoring joint '%s'.", joint_name.c_str());
    joints_.push_back(joint);
  }
  return true;
}

bool JointLimitMonitorPlugin::init(rclcpp::Node::SharedPtr node,
                                   const mjModel *model, mjData *data) {
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  const std::string param_prefix =
      "mujoco_plugins." + node_->get_sub_namespace() + ".";
  try {
    violation_duration_ =
        get_optional_double(node_, param_prefix + "violation_duration")
            .value_or(0.5);
    if (!register_joints(model)) {
      return false;
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(logger_, "Failed to parse parameters: %s", ex.what());
    return false;
  }

  const std::string topic_param = param_prefix + "topic";
  if (!node_->has_parameter(topic_param)) {
    node_->declare_parameter(topic_param, "joint_limit_violations");
  }
  const std::string topic = node_->get_parameter(topic_param).as_string();

  // Transient local, so that late subscribers still receive the latched report.
  violation_publisher_ = node_->create_publisher<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).reliable().transient_local());

  last_time_ = data->time;

  RCLCPP_INFO(logger_,
              "JointLimitMonitorPlugin monitoring %zu joint(s), publishing "
              "violations to '%s'.",
              joints_.size(), violation_publisher_->get_topic_name());
  return true;
}

bool JointLimitMonitorPlugin::check_limit(const std::string &joint_name,
                                          const std::string &limit_name,
                                          double limit, double value,
                                          bool is_violated, double time,
                                          LimitState &state) {
  if (state.latched) {
    return false;
  }

  if (!is_violated) {
    state.violation_start.reset();
    return false;
  }

  if (!state.violation_start) {
    state.violation_start = time;
  }

  if (time - *state.violation_start < violation_duration_) {
    return false;
  }

  state.latched = true;

  std::ostringstream violation;
  violation << joint_name << ": " << limit_name << " " << limit
            << " violated with value " << value << " (since sim time "
            << *state.violation_start << " s)";
  violations_.push_back(violation.str());
  RCLCPP_ERROR(logger_, "Joint limit violation: %s", violation.str().c_str());
  return true;
}

void JointLimitMonitorPlugin::update(const mjModel * /*model*/, mjData *data) {
  const double time = data->time;

  // A world reset rewinds the sim clock. Restart any in-progress violations,
  // but keep latched ones.
  if (time < last_time_) {
    for (auto &joint : joints_) {
      joint.min_position_state.violation_start.reset();
      joint.max_position_state.violation_start.reset();
      joint.max_velocity_state.violation_start.reset();
      joint.max_acceleration_state.violation_start.reset();
    }
  }
  last_time_ = time;

  bool new_violation = false;
  for (auto &joint : joints_) {
    const double position = data->qpos[joint.qpos_adr];
    const double velocity = data->qvel[joint.dof_adr];
    const double acceleration = data->qacc[joint.dof_adr];

    if (joint.min_position) {
      new_violation |= check_limit(
          joint.name, "min_position", *joint.min_position, position,
          position < *joint.min_position, time, joint.min_position_state);
    }
    if (joint.max_position) {
      new_violation |= check_limit(
          joint.name, "max_position", *joint.max_position, position,
          position > *joint.max_position, time, joint.max_position_state);
    }
    if (joint.max_velocity) {
      new_violation |=
          check_limit(joint.name, "max_velocity", *joint.max_velocity, velocity,
                      std::abs(velocity) > *joint.max_velocity, time,
                      joint.max_velocity_state);
    }
    if (joint.max_acceleration) {
      new_violation |= check_limit(
          joint.name, "max_acceleration", *joint.max_acceleration, acceleration,
          std::abs(acceleration) > *joint.max_acceleration, time,
          joint.max_acceleration_state);
    }
  }

  if (new_violation) {
    std_msgs::msg::String msg;
    msg.data = get_violation_report();
    violation_publisher_->publish(msg);
  }
}

std::string JointLimitMonitorPlugin::get_violation_report() const {
  std::string report;
  for (const auto &violation : violations_) {
    report += violation + "\n";
  }
  return report;
}

void JointLimitMonitorPlugin::cleanup() {
  violation_publisher_.reset();
  node_.reset();
}

} // namespace clr_mujoco_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(clr_mujoco_plugins::JointLimitMonitorPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
