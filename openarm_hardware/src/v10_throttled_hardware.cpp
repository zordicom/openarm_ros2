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
#include "openarm/realtime/can_transport.hpp"
#include "openarm/realtime/canfd_transport.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace openarm_hardware {

OpenArm_v10ThrottledHardware::OpenArm_v10ThrottledHardware() {
  // Initialize timestamps
  auto now = std::chrono::steady_clock::now();
  last_stats_log_ = now;
  last_partial_write_warn_ = now;
  last_partial_read_warn_ = now;
  last_no_data_warn_ = now;

  // Initialize per-motor timestamps to epoch (will be updated on first write)
  std::fill(last_motor_write_.begin(), last_motor_write_.end(),
            std::chrono::steady_clock::time_point());
  std::fill(motor_write_interval_us_.begin(), motor_write_interval_us_.end(), 0);
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

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
      "Successfully initialized throttled hardware interface with %zu joints",
      num_joints_);

  return CallbackReturn::SUCCESS;
}

OpenArm_v10ThrottledHardware::CallbackReturn
OpenArm_v10ThrottledHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Create appropriate transport (CAN or CAN-FD)
  try {
    std::unique_ptr<openarm::realtime::IOpenArmTransport> transport;
    if (controller_config_.can_fd) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                  "Initializing with CAN-FD transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::CANFDTransport>(config_.can_interface);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                  "Initializing with standard CAN transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::CANTransport>(config_.can_interface);
    }

    // Create RT-safe OpenArm interface with transport
    openarm_rt_ = std::make_unique<openarm::realtime::OpenArm>(std::move(transport));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to initialize OpenArm: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Add motors to RT-safe wrapper based on configuration
  for (size_t i = 0; i < controller_config_.arm_joints.size(); i++) {
    const auto& motor = controller_config_.arm_joints[i];
    int motor_idx = openarm_rt_->add_motor(motor.type, motor.send_can_id,
                                           motor.recv_can_id);
    if (motor_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to add motor %s to RT-safe wrapper",
                   motor.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Calculate write interval from update rate: interval_us = 1,000,000 / rate_hz
    motor_write_interval_us_[i] = static_cast<int64_t>(1000000.0 / motor.update_rate_hz);

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "Added motor: %s (send: 0x%03X, recv: 0x%03X, rate: %.1f Hz, interval: %ld us)",
                motor.name.c_str(), motor.send_can_id, motor.recv_can_id,
                motor.update_rate_hz, motor_write_interval_us_[i]);
  }

  // Add gripper motor if configured
  if (controller_config_.gripper_joint.has_value()) {
    size_t gripper_idx_local = controller_config_.arm_joints.size();
    int gripper_idx =
        openarm_rt_->add_motor(controller_config_.gripper_joint->motor_type,
                               controller_config_.gripper_joint->send_can_id,
                               controller_config_.gripper_joint->recv_can_id);
    if (gripper_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                   "Failed to add gripper motor to RT-safe wrapper");
      return CallbackReturn::ERROR;
    }

    // Calculate write interval from update rate
    motor_write_interval_us_[gripper_idx_local] =
        static_cast<int64_t>(1000000.0 / controller_config_.gripper_joint->update_rate_hz);

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                "Added gripper motor (rate: %.1f Hz, interval: %ld us)",
                controller_config_.gripper_joint->update_rate_hz,
                motor_write_interval_us_[gripper_idx_local]);
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

  size_t enabled = openarm_rt_->enable_all_motors_rt(5000);
  if (enabled != num_joints_) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to enable all motors: %zu/%zu", enabled,
                 openarm_rt_->get_motor_count());
    return CallbackReturn::ERROR;
  }

  // Initialize last write time for each motor so we start writing immediately
  auto now = std::chrono::steady_clock::now();
  for (size_t i = 0; i < num_joints_; ++i) {
    last_motor_write_[i] = now;
  }

  // Add small delay to allow motors to fully initialize after enable
  auto delay_start = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - delay_start)
             .count() < 100) {
    // Busy wait for 100ms
  }

  size_t received = openarm_rt_->receive_states_batch_rt(
      motor_states_.data(), openarm_rt_->get_motor_count(), 5000);

  if (received != num_joints_) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to get initial state for all motors: %zu/%zu",
                 received, num_joints_);
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < received; i++) {
    if (motor_states_[i].valid) {
      pos_states_[i] = motor_states_[i].position;
      vel_states_[i] = motor_states_[i].velocity;
      tau_states_[i] = motor_states_[i].torque;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                  "Initialized joint %zu: pos=%.3f, vel=%.3f, tau=%.3f", i,
                  pos_states_[i], vel_states_[i], tau_states_[i]);
    }
  }

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
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  stats_.read_count++;

  // Periodically log stats
  auto now = std::chrono::steady_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_)
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

  auto now = std::chrono::steady_clock::now();

  // Use controller period as timeout (convert to microseconds)
  int timeout_us = static_cast<int>(period.seconds() * 1e6);

  size_t sent = 0;
  size_t motors_to_send = 0;

  // Build list of motors that need to be updated this cycle based on their individual rates
  for (size_t i = 0; i < num_joints_; i++) {
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                          now - last_motor_write_[i])
                          .count();

    // Check if it's time to update this motor
    if (elapsed_us >= motor_write_interval_us_[i]) {
      motors_to_update_[motors_to_send++] = i;
      last_motor_write_[i] = now;
    }
  }

  // Send commands based on current mode (only for motors that need updates)
  if (current_mode_ == ControlMode::MIT) {
    // Pack MIT commands for motors that need updates
    for (size_t j = 0; j < motors_to_send; j++) {
      size_t i = motors_to_update_[j];
      mit_params_[j].q = pos_commands_[i];
      mit_params_[j].dq = vel_commands_[i];
      mit_params_[j].tau = tau_commands_[i];

      // Use arm joint gains or gripper gains
      if (i < controller_config_.arm_joints.size()) {
        mit_params_[j].kp = controller_config_.arm_joints[i].kp;
        mit_params_[j].kd = controller_config_.arm_joints[i].kd;
      } else if (controller_config_.gripper_joint.has_value()) {
        mit_params_[j].kp = controller_config_.gripper_joint->kp;
        mit_params_[j].kd = controller_config_.gripper_joint->kd;
      }
    }

    // Send MIT commands (batch for motors that need updates)
    if (motors_to_send > 0) {
      stats_.can_writes++;
      sent = openarm_rt_->send_mit_batch_rt(mit_params_.data(), motors_to_send, timeout_us);
      if (sent < motors_to_send) {
        stats_.tx_partial++;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_partial_write_warn_)
                           .count();
        if (elapsed >= WARN_THROTTLE_MS) {
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                      "Partial MIT write: sent %zu/%zu commands", sent, motors_to_send);
          last_partial_write_warn_ = now;
        }
      }
    }

  } else if (current_mode_ == ControlMode::POSITION_VELOCITY) {
    // Pack position/velocity commands for motors that need updates
    for (size_t j = 0; j < motors_to_send; j++) {
      size_t i = motors_to_update_[j];
      posvel_params_[j].q = pos_commands_[i];
      posvel_params_[j].dq = vel_commands_[i];
    }

    // Send position/velocity commands (batch for motors that need updates)
    if (motors_to_send > 0) {
      stats_.can_writes++;
      sent = openarm_rt_->send_posvel_batch_rt(posvel_params_.data(), motors_to_send,
                                        timeout_us);
      if (sent < motors_to_send) {
        stats_.tx_partial++;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_partial_write_warn_)
                           .count();
        if (elapsed >= WARN_THROTTLE_MS) {
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                      "Partial pos/vel write: sent %zu/%zu commands", sent, motors_to_send);
          last_partial_write_warn_ = now;
        }
      }
    }
  } else {
    // No active controller - send refresh command for motors that need updates
    if (motors_to_send > 0) {
      stats_.can_writes++;
      openarm_rt_->refresh_all_motors_rt(timeout_us);
    }
  }

  // After sending commands, read back the motor states
  size_t received = openarm_rt_->receive_states_batch_rt(
      motor_states_.data(), num_joints_, timeout_us);

  if (received > 0) {
    stats_.can_reads++;
    stats_.rx_received += received;

    // Log partial reads (RT-safe throttling)
    if (received < num_joints_) {
      stats_.rx_partial++;
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - last_partial_read_warn_)
                         .count();
      if (elapsed >= WARN_THROTTLE_MS) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                    "Partial read: received %zu/%zu motor states", received, num_joints_);
        last_partial_read_warn_ = now;
      }
    }

    // Update cached states
    for (size_t i = 0; i < received && i < num_joints_; i++) {
      if (motor_states_[i].valid) {
        pos_states_[i] = motor_states_[i].position;
        vel_states_[i] = motor_states_[i].velocity;
        tau_states_[i] = motor_states_[i].torque;
      }
    }
  } else {
    stats_.rx_no_data++;
    // Log no data (RT-safe throttling)
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - last_no_data_warn_)
                       .count();
    if (elapsed >= WARN_THROTTLE_MS) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                  "No motor states received in write cycle");
      last_no_data_warn_ = now;
    }
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
      motor_config.kp =
          (kp_it != joint.parameters.end()) ? std::stod(kp_it->second) : 0.0;
      motor_config.kd =
          (kd_it != joint.parameters.end()) ? std::stod(kd_it->second) : 0.0;

      // Parse update rate
      auto rate_it = joint.parameters.find("update_rate_hz");
      motor_config.update_rate_hz =
          (rate_it != joint.parameters.end()) ? std::stod(rate_it->second) : 150.0;

      controller_config_.arm_joints.push_back(motor_config);

      RCLCPP_INFO(logger,
                  "Configured arm joint: %s (type=%d, kp=%.2f, kd=%.2f, rate=%.1f Hz)",
                  joint.name.c_str(), static_cast<int>(motor_config.type),
                  motor_config.kp, motor_config.kd, motor_config.update_rate_hz);
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

  // Switch motors to MIT control mode
  if (!openarm_rt_->set_mode_all_rt(openarm::realtime::ControlMode::MIT, 1000)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to set MIT mode on motors");
    return false;
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

  // Switch motors to position/velocity control mode
  if (!openarm_rt_->set_mode_all_rt(openarm::realtime::ControlMode::POSITION_VELOCITY, 1000)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10ThrottledHardware"),
                 "Failed to set position/velocity mode on motors");
    return false;
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
  bool has_effort = false;

  for (const auto& interface : interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
      has_position = true;
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
  RCLCPP_INFO(logger, "  Batch writes:  %lu (partial: %lu)",
              stats_.can_writes, stats_.tx_partial);
  RCLCPP_INFO(logger, "  Batch reads:   %lu (received: %lu frames, partial: %lu, no-data: %lu)",
              stats_.can_reads, stats_.rx_received, stats_.rx_partial, stats_.rx_no_data);

  // Highlight performance issues
  if (stats_.tx_partial > 0 || stats_.rx_partial > 0 || stats_.rx_no_data > 0) {
    RCLCPP_WARN(logger, "Performance issues detected:");
    if (stats_.tx_partial > 0) {
      double tx_partial_pct = 100.0 * stats_.tx_partial / std::max(stats_.can_writes, 1UL);
      RCLCPP_WARN(logger, "  Partial writes: %lu (%.1f%% of writes)",
                  stats_.tx_partial, tx_partial_pct);
    }
    if (stats_.rx_partial > 0) {
      double rx_partial_pct = 100.0 * stats_.rx_partial / std::max(stats_.can_reads, 1UL);
      RCLCPP_WARN(logger, "  Partial reads: %lu (%.1f%% of reads)",
                  stats_.rx_partial, rx_partial_pct);
    }
    if (stats_.rx_no_data > 0) {
      double no_data_pct = 100.0 * stats_.rx_no_data / std::max(stats_.write_count, 1UL);
      RCLCPP_WARN(logger, "  No data received: %lu (%.1f%% of write cycles)",
                  stats_.rx_no_data, no_data_pct);
    }
  }

  // Log per-motor refresh rates
  RCLCPP_INFO(logger, "Motor refresh rates (configured):");
  for (size_t i = 0; i < num_joints_; i++) {
    double rate_hz = 1000000.0 / motor_write_interval_us_[i];
    RCLCPP_INFO(logger, "  Motor %zu: %.1f Hz (interval: %ld us)",
                i, rate_hz, motor_write_interval_us_[i]);
  }

  // Reset stats for next interval
  stats_ = Stats();
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10ThrottledHardware,
                       hardware_interface::SystemInterface)
