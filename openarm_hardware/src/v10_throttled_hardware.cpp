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

#include "openarm_hardware/v10_throttled_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace openarm_hardware {

OpenArm_v10ThrottledHardware::OpenArm_v10ThrottledHardware() {
  // Initialize timestamps
  last_can_write_ = std::chrono::steady_clock::now();
  last_stats_log_ = std::chrono::steady_clock::now();
}

OpenArm_v10ThrottledHardware::CallbackReturn
OpenArm_v10ThrottledHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parse configuration
  if (!parse_config(info)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to parse hardware configuration");
    return CallbackReturn::ERROR;
  }

  // Validate joint count
  if (num_joints_ > MAX_JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Number of joints (%zu) exceeds maximum supported (%zu)",
                 num_joints_, MAX_JOINTS);
    return CallbackReturn::ERROR;
  }

  // Initialize state and command arrays
  std::fill(pos_states_.begin(), pos_states_.end(), 0.0);
  std::fill(vel_states_.begin(), vel_states_.end(), 0.0);
  std::fill(tau_states_.begin(), tau_states_.end(), 0.0);
  std::fill(pos_commands_.begin(), pos_commands_.end(), 0.0);
  std::fill(vel_commands_.begin(), vel_commands_.end(), 0.0);
  std::fill(tau_commands_.begin(), tau_commands_.end(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Successfully initialized throttled hardware interface with %zu joints",
              num_joints_);

  return CallbackReturn::SUCCESS;
}

OpenArm_v10ThrottledHardware::CallbackReturn
OpenArm_v10ThrottledHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Create RT-safe OpenArm interface
  openarm_rt_ = std::make_unique<openarm::realtime::OpenArm>();

  if (!openarm_rt_->init(config_.can_interface)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to initialize RT-safe OpenArm interface");
    return CallbackReturn::ERROR;
  }

  // Add motors to RT-safe wrapper based on configuration
  for (const auto& motor : controller_config_.arm_joints) {
    int motor_idx = openarm_rt_->add_motor(motor.type, motor.send_can_id,
                                           motor.recv_can_id);
    if (motor_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to add motor %s to RT-safe wrapper",
                   motor.name.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "Added motor: %s (send: 0x%03X, recv: 0x%03X)",
                motor.name.c_str(), motor.send_can_id, motor.recv_can_id);
  }

  // Add gripper motor if configured
  if (controller_config_.gripper_joint.has_value()) {
    int gripper_idx =
        openarm_rt_->add_motor(controller_config_.gripper_joint->motor_type,
                               controller_config_.gripper_joint->send_can_id,
                               controller_config_.gripper_joint->recv_can_id);
    if (gripper_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to add gripper motor to RT-safe wrapper");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "Added gripper motor");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Hardware interface configured successfully");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10ThrottledHardware::CallbackReturn
OpenArm_v10ThrottledHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Check if RT kernel is available
  if (!realtime_tools::has_realtime_kernel()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "RT kernel not detected, running without RT guarantees");
  }

  // Enable motors
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Enabling motors on activation");

  size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
  if (enabled != openarm_rt_->get_motor_count()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to enable all motors: %zu/%zu", enabled,
                 openarm_rt_->get_motor_count());
    return CallbackReturn::ERROR;
  }

  // Initialize last write time to now so we start writing immediately
  last_can_write_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Hardware interface activated successfully. Motors enabled.");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10ThrottledHardware::CallbackReturn
OpenArm_v10ThrottledHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Disable motors
  if (openarm_rt_ && openarm_rt_->is_ready()) {
    openarm_rt_->disable_all_motors_rt(1000);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Hardware interface deactivated");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10ThrottledHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_joints_; i++) {
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    state_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10ThrottledHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_joints_; i++) {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type OpenArm_v10ThrottledHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  stats_.read_count++;

  // Use controller period as timeout (convert to microseconds)
  int timeout_us = static_cast<int>(period.seconds() * 1e6);

  // ALWAYS try to read from CAN (non-blocking, returns immediately if no data)
  std::array<openarm::damiao_motor::StateResult, MAX_JOINTS> motor_states;
  size_t received = openarm_rt_->receive_states_batch_rt(
      motor_states.data(), num_joints_, timeout_us);

  if (received > 0) {
    stats_.can_reads++;
    stats_.rx_received += received;

    // Update cached states
    for (size_t i = 0; i < received && i < num_joints_; i++) {
      if (motor_states[i].valid) {
        pos_states_[i] = motor_states[i].position;
        vel_states_[i] = motor_states[i].velocity;
        tau_states_[i] = motor_states[i].torque;
      }
    }
  } else {
    stats_.rx_no_data++;
  }

  // Periodically log stats
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                     now - last_stats_log_)
                     .count();
  if (elapsed >= STATS_LOG_INTERVAL_SEC) {
    log_stats();
    last_stats_log_ = now;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10ThrottledHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  stats_.write_count++;

  // Check if we should throttle this write
  auto now = std::chrono::steady_clock::now();
  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        now - last_can_write_)
                        .count();

  if (elapsed_us < CAN_WRITE_INTERVAL_US) {
    // Skip this write cycle - too soon since last write
    stats_.tx_skipped++;
    return hardware_interface::return_type::OK;
  }

  // Time to write to CAN bus
  stats_.can_writes++;
  last_can_write_ = now;

  // Use controller period as timeout (convert to microseconds)
  int timeout_us = static_cast<int>(period.seconds() * 1e6);

  // Send commands based on current mode
  if (current_mode_ == ControlMode::MIT) {
    // Pack MIT commands
    for (size_t i = 0; i < num_joints_; i++) {
      mit_params_[i].q = pos_commands_[i];
      mit_params_[i].dq = vel_commands_[i];
      mit_params_[i].tau = tau_commands_[i];
      mit_params_[i].kp = controller_config_.arm_joints[i].kp;
      mit_params_[i].kd = controller_config_.arm_joints[i].kd;
    }
    // Send MIT commands (batch)
    openarm_rt_->send_mit_batch_rt(mit_params_.data(), num_joints_,
                                   timeout_us);

  } else if (current_mode_ == ControlMode::POSITION_VELOCITY) {
    // Pack position/velocity commands
    for (size_t i = 0; i < num_joints_; i++) {
      posvel_params_[i].q = pos_commands_[i];
      posvel_params_[i].dq = vel_commands_[i];
    }
    // Send position/velocity commands (batch)
    openarm_rt_->send_posvel_batch_rt(posvel_params_.data(), num_joints_,
                                      timeout_us);
  } else {
    // No active controller - send refresh command
    openarm_rt_->refresh_all_motors_rt(timeout_us);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10ThrottledHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/) {
  // Determine the new mode
  ControlMode new_mode = determine_mode_from_interfaces(start_interfaces);

  if (new_mode == ControlMode::UNINITIALIZED) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Cannot determine control mode from interfaces");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10ThrottledHardware::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& /*stop_interfaces*/) {
  ControlMode new_mode = determine_mode_from_interfaces(start_interfaces);

  if (new_mode == current_mode_) {
    return hardware_interface::return_type::OK;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
              "Switching control mode from %d to %d",
              static_cast<int>(current_mode_), static_cast<int>(new_mode));

  bool success = false;
  if (new_mode == ControlMode::MIT) {
    success = switch_to_mit_mode();
  } else if (new_mode == ControlMode::POSITION_VELOCITY) {
    success = switch_to_position_mode();
  }

  if (success) {
    current_mode_ = new_mode;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "Control mode switch completed successfully");
    return hardware_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to switch control mode");
    return hardware_interface::return_type::ERROR;
  }
}

bool OpenArm_v10ThrottledHardware::parse_config(
    const hardware_interface::HardwareInfo& info) {
  auto logger = rclcpp::get_logger("OpenArm_v10ThrottledHardware");

  // Parse CAN interface
  auto it = info.hardware_parameters.find("can_interface");
  if (it != info.hardware_parameters.end()) {
    config_.can_interface = it->second;
  }

  // Parse CAN timeout
  it = info.hardware_parameters.find("can_timeout_us");
  if (it != info.hardware_parameters.end()) {
    config_.can_timeout_us = std::stoi(it->second);
  }

  RCLCPP_INFO(logger, "CAN Interface: %s", config_.can_interface.c_str());
  RCLCPP_INFO(logger, "CAN Timeout: %d us", config_.can_timeout_us);

  // Parse joint configuration from URDF
  try {
    for (const auto& joint : info.joints) {
      MotorConfig motor_config;
      motor_config.name = joint.name;

      // Parse motor type
      auto type_it = joint.parameters.find("motor_type");
      if (type_it == joint.parameters.end()) {
        RCLCPP_ERROR(logger, "Missing motor_type for joint %s",
                     joint.name.c_str());
        return false;
      }
      motor_config.type = parse_motor_type_param(type_it->second);

      // Parse CAN IDs
      auto send_id_it = joint.parameters.find("send_can_id");
      auto recv_id_it = joint.parameters.find("recv_can_id");
      if (send_id_it == joint.parameters.end() ||
          recv_id_it == joint.parameters.end()) {
        RCLCPP_ERROR(logger, "Missing CAN IDs for joint %s",
                     joint.name.c_str());
        return false;
      }
      motor_config.send_can_id = std::stoul(send_id_it->second, nullptr, 0);
      motor_config.recv_can_id = std::stoul(recv_id_it->second, nullptr, 0);

      // Parse MIT gains
      auto kp_it = joint.parameters.find("kp");
      auto kd_it = joint.parameters.find("kd");
      motor_config.kp = (kp_it != joint.parameters.end())
                            ? std::stod(kp_it->second)
                            : 0.0;
      motor_config.kd = (kd_it != joint.parameters.end())
                            ? std::stod(kd_it->second)
                            : 0.0;

      controller_config_.arm_joints.push_back(motor_config);

      RCLCPP_INFO(logger, "Configured arm joint: %s (type=%d, kp=%.2f, kd=%.2f)",
                  joint.name.c_str(), static_cast<int>(motor_config.type),
                  motor_config.kp, motor_config.kd);
    }

    if (controller_config_.arm_joints.empty()) {
      RCLCPP_ERROR(logger, "No arm joints configured");
      return false;
    }

    // Build joint names vector
    joint_names_.clear();
    for (const auto& motor : controller_config_.arm_joints) {
      joint_names_.push_back(motor.name);
    }
    if (controller_config_.gripper_joint.has_value()) {
      joint_names_.push_back(controller_config_.gripper_joint->name);
    }
    num_joints_ = joint_names_.size();

    RCLCPP_INFO(logger, "Configured %zu total joints",
                controller_config_.arm_joints.size());

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to parse configuration: %s", e.what());
    return false;
  }
}

bool OpenArm_v10ThrottledHardware::switch_to_mit_mode() {
  // Enable motors if not already enabled
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
    if (enabled != openarm_rt_->get_motor_count()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to enable all motors for MIT mode");
      return false;
    }
  }

  // Initialize commands to current positions to avoid jumps
  for (size_t i = 0; i < num_joints_; i++) {
    pos_commands_[i] = pos_states_[i];
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
  }

  return true;
}

bool OpenArm_v10ThrottledHardware::switch_to_position_mode() {
  // Enable motors if not already enabled
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
    if (enabled != openarm_rt_->get_motor_count()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to enable all motors for position mode");
      return false;
    }
  }

  // Initialize commands to current positions
  for (size_t i = 0; i < num_joints_; i++) {
    pos_commands_[i] = pos_states_[i];
    vel_commands_[i] = 0.0;
  }

  return true;
}

ControlMode OpenArm_v10ThrottledHardware::determine_mode_from_interfaces(
    const std::vector<std::string>& interfaces) {
  bool has_position = false;
  bool has_velocity = false;
  bool has_effort = false;

  for (const auto& interface : interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      has_position = true;
    }
    if (interface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      has_velocity = true;
    }
    if (interface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
      has_effort = true;
    }
  }

  // MIT mode: effort interface claimed
  if (has_effort) {
    return ControlMode::MIT;
  }
  // Position-velocity mode: position interface claimed
  else if (has_position) {
    return ControlMode::POSITION_VELOCITY;
  }

  return ControlMode::UNINITIALIZED;
}

void OpenArm_v10ThrottledHardware::log_stats() {
  auto logger = rclcpp::get_logger("OpenArm_v10ThrottledHardware");

  RCLCPP_INFO(logger, "=== Throttled Hardware Stats (last %d sec) ===",
              STATS_LOG_INTERVAL_SEC);
  RCLCPP_INFO(logger, "Controller calls:");
  RCLCPP_INFO(logger, "  read():  %lu calls", stats_.read_count);
  RCLCPP_INFO(logger, "  write(): %lu calls", stats_.write_count);
  RCLCPP_INFO(logger, "CAN operations:");
  RCLCPP_INFO(logger, "  Writes:  %lu (skipped: %lu due to throttling)",
              stats_.can_writes, stats_.tx_skipped);
  RCLCPP_INFO(logger, "  Reads:   %lu attempts (received: %lu frames, no-data: %lu)",
              stats_.can_reads, stats_.rx_received, stats_.rx_no_data);

  if (stats_.write_count > 0) {
    double write_rate = stats_.can_writes * 1000.0 / (STATS_LOG_INTERVAL_SEC * 1000.0);
    double throttle_percent = 100.0 * stats_.tx_skipped / stats_.write_count;
    RCLCPP_INFO(logger, "Actual CAN write rate: %.1f Hz (%.1f%% throttled)",
                write_rate, throttle_percent);
  }

  // Reset stats for next interval
  stats_ = Stats();
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10ThrottledHardware,
                       hardware_interface::SystemInterface)
