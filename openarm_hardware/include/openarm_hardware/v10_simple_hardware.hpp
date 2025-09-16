// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief Configuration for a single motor
 */
struct MotorConfig {
  std::string name;
  openarm::damiao_motor::MotorType type;
  uint32_t send_can_id;
  uint32_t recv_can_id;
  double kp;
  double kd;
};

/**
 * @brief Configuration for the gripper
 */
struct GripperConfig {
  std::string name;
  openarm::damiao_motor::MotorType motor_type;
  uint32_t send_can_id;
  uint32_t recv_can_id;
  double kp;
  double kd;
  double closed_position;
  double open_position;
  double motor_closed_radians;
  double motor_open_radians;
};

struct ControllerConfig {
  // Which interface to connect to.
  std::string can_iface;

  // Use CAN FD instead of standard CAN.
  bool can_fd;

  // Configuration storage using POD types
  std::vector<MotorConfig> arm_joints;
  std::optional<GripperConfig> gripper_joint;
};

double gripper_joint_to_motor_radians(const GripperConfig& config,
                                      double joint_value);
double gripper_motor_radians_to_joint(const GripperConfig& config,
                                      double motor_radians);

/**
 * @brief Simplified OpenArm V10 Hardware Interface
 *
 * This is a simplified version that uses the OpenArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class OpenArm_v10HW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10HW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // ROS2 control joint names, state, and command vectors.
  std::vector<std::string> joint_names_;
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  bool generate_joint_names();
  bool load_motor_config_from_yaml(const std::string& yaml_file);

  ControllerConfig config_;
  std::string motor_config_file_;
};

}  // namespace openarm_hardware

// YAML conversion templates for our configuration structs
namespace YAML {
template <>
struct convert<openarm_hardware::MotorConfig> {
  static bool decode(const Node& node, openarm_hardware::MotorConfig& config);
};

template <>
struct convert<openarm_hardware::GripperConfig> {
  static bool decode(const Node& node, openarm_hardware::GripperConfig& config);
};

template <>
struct convert<openarm_hardware::ControllerConfig> {
  static bool decode(const Node& node,
                     openarm_hardware::ControllerConfig& config);
};
}  // namespace YAML
