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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "joint_limit_monitor_plugin.hpp"

namespace {
constexpr const char *kMjcf = R"(
<mujoco model="joint_limit_monitor_test">
  <worldbody>
    <body name="link1">
      <joint name="hinge_joint" type="hinge" axis="0 0 1"/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="link2">
        <joint name="slide_joint" type="slide" axis="1 0 0"/>
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
    </body>
    <body name="free_body" pos="1 0 0">
      <freejoint name="free_joint"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";

constexpr double kTimestep = 0.01;
} // namespace

class JointLimitMonitorPluginTest : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("joint_limit_monitor_test_node");
    plugin_node_ = node_->create_sub_node("joint_limit_monitor");

    char error[1024] = {0};
    mjSpec *spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    model_ = mj_compile(spec, nullptr);
    mj_deleteSpec(spec);
    ASSERT_NE(model_, nullptr);
    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);

    hinge_ = mj_name2id(model_, mjOBJ_JOINT, "hinge_joint");
    slide_ = mj_name2id(model_, mjOBJ_JOINT, "slide_joint");
  }

  void TearDown() override {
    plugin_node_.reset();
    node_.reset();
    mj_deleteData(data_);
    mj_deleteModel(model_);
  }

  /// Pre-declares and sets a parameter on plugin_node_ before init(), matching
  /// how launch parameter files set plugin parameters ahead of
  /// load_mujoco_plugins().
  template <typename T> void setParam(const std::string &name, const T &value) {
    const std::string resolved_name =
        "mujoco_plugins." + plugin_node_->get_sub_namespace() + "." + name;
    if (!plugin_node_->has_parameter(resolved_name)) {
      plugin_node_->declare_parameter(resolved_name,
                                      rclcpp::ParameterValue(value));
    }
    plugin_node_->set_parameter(rclcpp::Parameter(resolved_name, value));
  }

  /// joint_limit_monitor:
  ///   violation_duration: 0.5
  ///   joints:
  ///     hinge_joint: { min_position: -1.0, max_position: 1.0,
  ///     max_velocity: 2.0, max_acceleration: 10.0 } slide_joint: {
  ///     max_position: 0.5 } free_joint: { max_velocity: 1.0 } missing_joint: {
  ///     max_velocity: 1.0 }
  void set_plugin_params() {
    setParam("violation_duration", 0.5);
    setParam("joints.hinge_joint.min_position", -1.0);
    setParam("joints.hinge_joint.max_position", 1.0);
    setParam("joints.hinge_joint.max_velocity", 2.0);
    // Integers are accepted for numeric limits.
    setParam("joints.hinge_joint.max_acceleration", int64_t{10});
    setParam("joints.slide_joint.max_position", 0.5);
    setParam("joints.free_joint.max_velocity", 1.0);
    setParam("joints.missing_joint.max_velocity", 1.0);
  }

  /// Calls update() once per timestep for the given duration, advancing the sim
  /// time.
  void run_for(clr_mujoco_plugins::JointLimitMonitorPlugin &plugin,
               double duration) {
    const int steps = static_cast<int>(std::round(duration / kTimestep));
    for (int i = 0; i < steps; ++i) {
      data_->time += kTimestep;
      plugin.update(model_, data_);
    }
  }

  double &hinge_qpos() { return data_->qpos[model_->jnt_qposadr[hinge_]]; }
  double &hinge_qvel() { return data_->qvel[model_->jnt_dofadr[hinge_]]; }
  double &hinge_qacc() { return data_->qacc[model_->jnt_dofadr[hinge_]]; }
  double &slide_qpos() { return data_->qpos[model_->jnt_qposadr[slide_]]; }

  mjModel *model_{nullptr};
  mjData *data_{nullptr};
  int hinge_{-1};
  int slide_{-1};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;
};

TEST_F(JointLimitMonitorPluginTest, RegistersOnlyValidJoints) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // free_joint is not 1-DOF and missing_joint is not in the model, so both are
  // skipped.
  const auto &joints = plugin.get_monitored_joints();
  ASSERT_EQ(joints.size(), 2u);
  EXPECT_EQ(joints[0].name, "hinge_joint");
  EXPECT_EQ(joints[0].min_position, -1.0);
  EXPECT_EQ(joints[0].max_position, 1.0);
  EXPECT_EQ(joints[0].max_velocity, 2.0);
  EXPECT_EQ(joints[0].max_acceleration, 10.0);
  EXPECT_EQ(joints[1].name, "slide_joint");
  EXPECT_EQ(joints[1].max_position, 0.5);
  EXPECT_FALSE(joints[1].min_position.has_value());
  plugin.cleanup();
}

TEST_F(JointLimitMonitorPluginTest, InvalidPositionLimitsFailInit) {
  setParam("joints.hinge_joint.min_position", 1.0);
  setParam("joints.hinge_joint.max_position", -1.0);
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
}

TEST_F(JointLimitMonitorPluginTest, NonNumericLimitFailsInit) {
  setParam("joints.hinge_joint.max_velocity", std::string("fast"));
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
}

TEST_F(JointLimitMonitorPluginTest, ShortViolationDoesNotLatch) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  hinge_qpos() = 1.5;
  run_for(plugin, 0.3);
  hinge_qpos() = 0.0;
  run_for(plugin, 0.1);
  // A new violation must again be sustained for the full horizon.
  hinge_qpos() = 1.5;
  run_for(plugin, 0.3);
  EXPECT_EQ(plugin.get_violation_report(), "");
  plugin.cleanup();
}

TEST_F(JointLimitMonitorPluginTest, SustainedViolationLatches) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  slide_qpos() = 0.6;
  run_for(plugin, 0.3);
  slide_qpos() = 0.8;
  run_for(plugin, 0.3);

  const std::string report = plugin.get_violation_report();
  EXPECT_NE(
      report.find("slide_joint: max_position 0.5 violated with value 0.8"),
      std::string::npos)
      << report;

  // Returning within limits does not clear the violation.
  slide_qpos() = 0.0;
  run_for(plugin, 1.0);
  EXPECT_EQ(plugin.get_violation_report(), report);
  plugin.cleanup();
}

TEST_F(JointLimitMonitorPluginTest, VelocityAndAccelerationUseMagnitude) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  hinge_qvel() = -2.5;
  hinge_qacc() = -12.0;
  run_for(plugin, 0.6);

  const std::string report = plugin.get_violation_report();
  EXPECT_NE(report.find("hinge_joint: max_velocity 2 violated with value -2.5"),
            std::string::npos)
      << report;
  EXPECT_NE(
      report.find("hinge_joint: max_acceleration 10 violated with value -12"),
      std::string::npos)
      << report;
  EXPECT_EQ(report.find("position"), std::string::npos) << report;
  plugin.cleanup();
}

TEST_F(JointLimitMonitorPluginTest, ResetDropsPendingViolations) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  hinge_qpos() = -1.5;
  run_for(plugin, 0.4);
  // World reset rewinds the clock; the violation must restart from the reset.
  data_->time = 0.0;
  run_for(plugin, 0.4);
  EXPECT_EQ(plugin.get_violation_report(), "");
  run_for(plugin, 0.2);
  EXPECT_NE(plugin.get_violation_report().find("hinge_joint: min_position"),
            std::string::npos);
  plugin.cleanup();
}

TEST_F(JointLimitMonitorPluginTest, PublishesLatchedReport) {
  set_plugin_params();
  clr_mujoco_plugins::JointLimitMonitorPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  hinge_qpos() = 1.5;
  run_for(plugin, 0.6);
  slide_qpos() = 1.0;
  run_for(plugin, 0.6);

  // Subscribe only after both violations were published, which relies on
  // transient local durability.
  std::string received;
  auto sub = node_->create_subscription<std_msgs::msg::String>(
      "joint_limit_monitor/joint_limit_violations",
      rclcpp::QoS(1).reliable().transient_local(),
      [&received](const std_msgs::msg::String &msg) { received = msg.data; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(50));
  }

  EXPECT_EQ(received, plugin.get_violation_report());
  EXPECT_NE(received.find("hinge_joint: max_position"), std::string::npos)
      << received;
  EXPECT_NE(received.find("slide_joint: max_position"), std::string::npos)
      << received;
  plugin.cleanup();
}
