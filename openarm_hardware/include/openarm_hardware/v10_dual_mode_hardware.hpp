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
#include <filesystem>
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
#include "openarm_hardware/hardware_config.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief Control mode for motors
 * Based on DM motor control modes
 */
enum class ControlMode {
  UNINITIALIZED = -1,
  MIT = 1,            // MIT mode (torque/impedance control)
  POSITION_VELOCITY = 2,  // Position-Velocity mode (0x100 + ID frame)
  VELOCITY = 3,       // Velocity mode (not implemented)
  TORQUE_POSITION = 4 // Torque-Position mode (not implemented)
};

/**
 * @brief Dual-mode OpenArm V10 Hardware Interface
 *
 * This hardware interface supports two control modes:
 * 1. Position Mode: When position interface is claimed (joint_trajectory_controller)
 * 2. MIT Mode: When effort interface is claimed (impedance controllers)
 *
 * The mode is automatically selected based on which command interfaces are claimed.
 */
class OpenArm_v10DualModeHW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10DualModeHW();

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

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

 private:
  // OpenArm instance
  std::unique_ptr<openarm::can::socket::OpenArm> openarm_;

  // Control mode state
  ControlMode current_mode_ = ControlMode::UNINITIALIZED;
  ControlMode pending_mode_ = ControlMode::UNINITIALIZED;

  // Track which interfaces are claimed
  bool position_interface_claimed_ = false;
  bool velocity_interface_claimed_ = false;
  bool effort_interface_claimed_ = false;

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

  // Mode switching helpers
  bool switch_to_position_mode();
  bool switch_to_mit_mode();
  ControlMode determine_mode_from_interfaces(
      const std::vector<std::string>& interfaces);

  // Position mode control
  bool send_position_commands();

  // MIT mode control
  bool send_mit_commands();

  ControllerConfig config_;
  std::string motor_config_file_;

  // Logging helpers
  void log_mode_switch(ControlMode from, ControlMode to);

  // Power monitoring and logging
  size_t power_query_counter_ = 0;
  static constexpr size_t POWER_QUERY_INTERVAL = 50;  // Query power every 50 cycles (~333ms at 150 Hz)
  std::ofstream power_log_file_;
  bool power_logging_enabled_ = false;
  rclcpp::Time power_log_start_time_;
};

}  // namespace openarm_hardware