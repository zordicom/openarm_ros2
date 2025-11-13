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

#include "openarm_hardware/v10_rt_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "openarm/realtime/can.hpp"
#include "openarm/realtime/canfd.hpp"
#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace openarm_hardware {

OpenArm_v10RTHardware::OpenArm_v10RTHardware() {
  // Initialize timestamps
  auto now = std::chrono::steady_clock::now();
  last_stats_log_ = now;
  last_partial_write_warn_ = now;
  last_partial_read_warn_ = now;
  last_no_data_warn_ = now;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parse configuration
  if (!parse_config(info)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to parse hardware configuration");
    return CallbackReturn::ERROR;
  }

  // Validate joint count
  if (num_joints_ > MAX_JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
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
  std::fill(kp_commands_.begin(), kp_commands_.end(), 0.0);
  std::fill(kd_commands_.begin(), kd_commands_.end(), 0.0);

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10RTHardware"),
      "Successfully initialized throttled hardware interface with %zu joints",
      num_joints_);

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    std::unique_ptr<openarm::realtime::IOpenArmTransport> transport;
    if (controller_config_.can_fd) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Initializing with CAN-FD transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::can::CANFDSocket>(
          config_.can_interface);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Initializing with standard CAN transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::can::CANSocket>(
          config_.can_interface);
    }

    openarm_rt_ =
        std::make_unique<openarm::realtime::OpenArm>(std::move(transport));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to initialize OpenArm: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Add motors to RT-safe wrapper based on configuration
  for (size_t i = 0; i < controller_config_.arm_joints.size(); i++) {
    const auto& motor = controller_config_.arm_joints[i];
    int motor_id = openarm_rt_->add_motor(motor.type, motor.send_can_id,
                                          motor.recv_can_id);
    if (motor_id < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to add motor %s to RT-safe wrapper",
                   motor.name.c_str());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Added motor: %s (motor_id=%d, joint_idx=%zu, send: 0x%03X, "
                "recv: 0x%03X)",
                motor.name.c_str(), motor_id, i, motor.send_can_id,
                motor.recv_can_id);
  }

  // Add gripper motor if configured
  if (controller_config_.gripper_joint.has_value()) {
    size_t gripper_joint_idx = controller_config_.arm_joints.size();
    int gripper_motor_id =
        openarm_rt_->add_motor(controller_config_.gripper_joint->motor_type,
                               controller_config_.gripper_joint->send_can_id,
                               controller_config_.gripper_joint->recv_can_id);
    if (gripper_motor_id < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to add gripper motor to RT-safe wrapper");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Added gripper motor (motor_id=%d, joint_idx=%zu)",
                gripper_motor_id, gripper_joint_idx);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface configured successfully");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Check if RT kernel is available
  if (!realtime_tools::has_realtime_kernel()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "RT kernel not detected, running without RT guarantees");
  }

  // Enable motors
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Enabling motors on activation");

  ssize_t enabled = openarm_rt_->enable_all_motors_rt(5000);
  if (enabled < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Enable motors send failed with errno %d: %s", errno,
                 strerror(errno));
    return CallbackReturn::ERROR;
  }
  if (enabled != static_cast<ssize_t>(num_joints_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to enable all motors: %zd/%zu", enabled,
                 openarm_rt_->get_motor_count());
    return CallbackReturn::ERROR;
  }

  // Add small delay to allow motors to fully initialize after enable
  auto delay_start = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - delay_start)
             .count() < 100) {
    // Busy wait for 100ms
  }

  ssize_t received = openarm_rt_->receive_states_batch_rt(
      motor_states_.data(), openarm_rt_->get_motor_count(), 5000);

  if (received != num_joints_) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to get initial state for all motors: %zd/%zd",
                 received, num_joints_);
    return CallbackReturn::ERROR;
  }

  // Switch motors to MIT control mode
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Setting all motors to MIT mode");
  if (!openarm_rt_->set_mode_all_rt(openarm::realtime::ControlMode::MIT, 1000)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to set MIT mode on motors");
    return CallbackReturn::ERROR;
  }

  // Initialize states and commands from received motor states
  for (ssize_t i = 0; i < received; i++) {
    if (motor_states_[i].valid) {
      pos_states_[i] = motor_states_[i].position;
      vel_states_[i] = motor_states_[i].velocity;
      tau_states_[i] = motor_states_[i].torque;

      // Initialize commands to current positions to avoid jumps
      pos_commands_[i] = pos_states_[i];
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
      kp_commands_[i] = 0.0;  // Will use defaults from config
      kd_commands_[i] = 0.0;  // Will use defaults from config

      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Initialized joint %zu: pos=%.3f, vel=%.3f, tau=%.3f, "
                  "default kp=%.2f, kd=%.2f",
                  i, pos_states_[i], vel_states_[i], tau_states_[i],
                  default_kp_[i], default_kd_[i]);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface activated successfully in MIT mode.");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Disable motors
  if (openarm_rt_) {
    openarm_rt_->disable_all_motors_rt(1000);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface deactivated");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10RTHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (ssize_t i = 0; i < num_joints_; i++) {
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
OpenArm_v10RTHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (ssize_t i = 0; i < num_joints_; i++) {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], "kp", &kp_commands_[i]);
    command_interfaces.emplace_back(joint_names_[i], "kd", &kd_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type OpenArm_v10RTHardware::read(
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

hardware_interface::return_type OpenArm_v10RTHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  stats_.write_count++;

  auto now = std::chrono::steady_clock::now();

  // Use controller period as timeout (convert to microseconds)
  int timeout_us = static_cast<int>(period.seconds() * 1e6);

  // Pack MIT commands for all motors
  for (ssize_t i = 0; i < num_joints_; i++) {
    mit_params_[i].q = pos_commands_[i];
    mit_params_[i].dq = vel_commands_[i];
    mit_params_[i].tau = tau_commands_[i];

    // Use commanded kp/kd if non-zero, otherwise use configured defaults
    mit_params_[i].kp = (kp_commands_[i] > 0.0) ? kp_commands_[i] : default_kp_[i];
    mit_params_[i].kd = (kd_commands_[i] > 0.0) ? kd_commands_[i] : default_kd_[i];
  }

  stats_.can_writes++;
  ssize_t sent = openarm_rt_->send_mit_batch_rt(mit_params_.data(), num_joints_,
                                                timeout_us);

  if (sent >= 0 && sent < num_joints_) {
    stats_.tx_partial++;
  }

  // Check for send errors
  if (sent < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "MIT control send failed with errno %d: %s", errno,
                 strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  // After sending commands, read back the motor states
  auto received = openarm_rt_->receive_states_batch_rt(motor_states_.data(),
                                                       num_joints_, timeout_us);

  if (received > 0) {
    stats_.can_reads++;
    stats_.rx_received += received;

    // Update cached states from received motor states
    // States are returned in motor order (matching the order they were added)
    for (ssize_t i = 0; i < received; i++) {
      if (motor_states_[i].valid) {
        // Check for unrecoverable errors (error codes 0x8-0xE)
        // 0x1 = no error, 0x8-0xE = unrecoverable errors
        uint8_t error_code = motor_states_[i].error_code;
        if (error_code != 0x1 && error_code >= 0x8 && error_code <= 0xE) {
          std::string error_msg = error_code_to_string(error_code);
          RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                       "Motor %zd has unrecoverable error: %s (0x%X). "
                       "Stopping controller.",
                       i, error_msg.c_str(), error_code);
          return hardware_interface::return_type::ERROR;
        }

        stats_.motor_receives[i]++;
        pos_states_[i] = motor_states_[i].position;
        vel_states_[i] = motor_states_[i].velocity;
        tau_states_[i] = motor_states_[i].torque;
      }
    }

    // Log partial reads (RT-safe throttling)
    if (received < num_joints_) {
      stats_.rx_partial++;
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - last_partial_read_warn_)
                         .count();
      if (elapsed >= WARN_THROTTLE_MS) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                    "Partial read: received %zu/%zu motor states", received,
                    num_joints_);
        last_partial_read_warn_ = now;
      }
    }
  } else {
    stats_.rx_no_data++;
    // Log no data (RT-safe throttling)
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - last_no_data_warn_)
                       .count();
    if (elapsed >= WARN_THROTTLE_MS) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "No motor states received in write cycle");
      last_no_data_warn_ = now;
    }
  }

  return hardware_interface::return_type::OK;
}

bool OpenArm_v10RTHardware::parse_config(
    const hardware_interface::HardwareInfo& info) {
  auto logger = rclcpp::get_logger("OpenArm_v10RTHardware");

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

  // Parse CAN FD flag
  it = info.hardware_parameters.find("can_fd");
  if (it != info.hardware_parameters.end()) {
    controller_config_.can_fd = parse_bool_param(it->second);
  } else {
    controller_config_.can_fd = false;  // Default to standard CAN
  }

  RCLCPP_INFO(logger, "CAN Interface: %s", config_.can_interface.c_str());
  RCLCPP_INFO(logger, "CAN Timeout: %d us", config_.can_timeout_us);
  RCLCPP_INFO(logger, "CAN FD: %s",
              controller_config_.can_fd ? "enabled" : "disabled");

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

      controller_config_.arm_joints.push_back(motor_config);

      RCLCPP_INFO(logger,
                  "Configured arm joint: %s (type=%d, kp=%.2f, kd=%.2f)",
                  joint.name.c_str(), static_cast<int>(motor_config.type),
                  motor_config.kp, motor_config.kd);
    }

    if (controller_config_.arm_joints.empty()) {
      RCLCPP_ERROR(logger, "No arm joints configured");
      return false;
    }

    // Build joint names vector and store default kp/kd values
    joint_names_.clear();
    for (size_t i = 0; i < controller_config_.arm_joints.size(); i++) {
      const auto& motor = controller_config_.arm_joints[i];
      joint_names_.push_back(motor.name);
      default_kp_[i] = motor.kp;
      default_kd_[i] = motor.kd;
    }
    if (controller_config_.gripper_joint.has_value()) {
      joint_names_.push_back(controller_config_.gripper_joint->name);
    }
    num_joints_ = static_cast<ssize_t>(joint_names_.size());

    RCLCPP_INFO(logger, "Configured %zu total joints",
                controller_config_.arm_joints.size());

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to parse configuration: %s", e.what());
    return false;
  }
}

void OpenArm_v10RTHardware::log_stats() {
  auto logger = rclcpp::get_logger("OpenArm_v10RTHardware");

  RCLCPP_INFO(logger, "=== Throttled Hardware Stats (last %d sec) ===",
              STATS_LOG_INTERVAL_SEC);
  RCLCPP_INFO(logger, "Controller calls:");
  RCLCPP_INFO(logger, "  read():  %lu calls", stats_.read_count);
  RCLCPP_INFO(logger, "  write(): %lu calls", stats_.write_count);

  RCLCPP_INFO(logger, "CAN operations:");
  RCLCPP_INFO(logger, "  Batch writes:  %lu (partial: %lu)", stats_.can_writes,
              stats_.tx_partial);
  RCLCPP_INFO(
      logger,
      "  Batch reads:   %lu (received: %lu frames, partial: %lu, no-data: %lu)",
      stats_.can_reads, stats_.rx_received, stats_.rx_partial,
      stats_.rx_no_data);

  // Highlight performance issues
  if (stats_.tx_partial > 0 || stats_.rx_partial > 0 || stats_.rx_no_data > 0) {
    RCLCPP_WARN(logger, "Performance issues detected:");
    if (stats_.tx_partial > 0) {
      double tx_partial_pct =
          100.0 * stats_.tx_partial / std::max(stats_.can_writes, 1UL);
      RCLCPP_WARN(logger, "  Partial writes: %lu (%.1f%% of writes)",
                  stats_.tx_partial, tx_partial_pct);
    }
    if (stats_.rx_partial > 0) {
      double rx_partial_pct =
          100.0 * stats_.rx_partial / std::max(stats_.can_reads, 1UL);
      RCLCPP_WARN(logger, "  Partial reads: %lu (%.1f%% of reads)",
                  stats_.rx_partial, rx_partial_pct);
    }
    if (stats_.rx_no_data > 0) {
      double no_data_pct =
          100.0 * stats_.rx_no_data / std::max(stats_.write_count, 1UL);
      RCLCPP_WARN(logger, "  No data received: %lu (%.1f%% of write cycles)",
                  stats_.rx_no_data, no_data_pct);
    }
  }

  // Log per-motor send/receive counts
  RCLCPP_INFO(logger, "Per-motor statistics:");
  for (ssize_t i = 0; i < num_joints_; i++) {
    double actual_send_rate =
        stats_.motor_sends[i] / (double)STATS_LOG_INTERVAL_SEC;
    double actual_recv_rate =
        stats_.motor_receives[i] / (double)STATS_LOG_INTERVAL_SEC;

    RCLCPP_INFO(logger,
                "  Motor %zu: sends=%lu (%.1f Hz), receives=%lu (%.1f Hz)", i,
                stats_.motor_sends[i], actual_send_rate,
                stats_.motor_receives[i], actual_recv_rate);
  }

  // Reset stats for next interval
  stats_ = Stats();
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10RTHardware,
                       hardware_interface::SystemInterface)
