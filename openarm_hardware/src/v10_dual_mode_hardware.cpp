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

#include "openarm_hardware/v10_dual_mode_hardware.hpp"
#include "openarm_hardware/config_yaml.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace openarm_hardware {

OpenArm_v10DualModeHW::OpenArm_v10DualModeHW() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Dual-Mode Hardware Interface Created");
}

hardware_interface::CallbackReturn OpenArm_v10DualModeHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Load motor configuration from YAML
  if (!load_motor_config_from_yaml(motor_config_file_)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on configuration
  if (!generate_joint_names()) {
    return CallbackReturn::ERROR;
  }

  // Log the joint count
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Configured with %zu joints total", joint_names_.size());

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              config_.can_iface.c_str(),
              config_.can_fd ? "enabled" : "disabled");

  try {
    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(
        config_.can_iface, config_.can_fd);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                 "Failed to initialize OpenArm on interface %s: %s",
                 config_.can_iface.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  // Build arrays from arm configs for initialization
  std::vector<openarm::damiao_motor::MotorType> arm_motor_types;
  std::vector<uint32_t> arm_send_can_ids;
  std::vector<uint32_t> arm_recv_can_ids;

  for (const auto& motor : config_.arm_joints) {
    arm_motor_types.push_back(motor.type);
    arm_send_can_ids.push_back(motor.send_can_id);
    arm_recv_can_ids.push_back(motor.recv_can_id);
  }

  // Initialize arm motors
  openarm_->init_arm_motors(arm_motor_types, arm_send_can_ids,
                            arm_recv_can_ids);

  // Initialize gripper if enabled
  if (config_.gripper_joint.has_value()) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Initializing gripper...");
    const auto& gripper = config_.gripper_joint.value();
    openarm_->init_gripper_motor(gripper.motor_type, gripper.send_can_id,
                                 gripper.recv_can_id);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "OpenArm V10 Dual-Mode HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10DualModeHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Don't set control mode here - wait for interface claiming
  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10DualModeHW"),
      "Hardware configured. Mode will be set on controller activation.");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10DualModeHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArm_v10DualModeHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type
OpenArm_v10DualModeHW::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Preparing command mode switch...");

  // Log what's being started and stopped
  if (!start_interfaces.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Starting interfaces:");
    for (const auto& interface : start_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"), "  - %s",
                  interface.c_str());
    }
  }

  if (!stop_interfaces.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Stopping interfaces:");
    for (const auto& interface : stop_interfaces) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"), "  - %s",
                  interface.c_str());
    }
  }

  // Determine new mode based on starting interfaces
  ControlMode new_mode = determine_mode_from_interfaces(start_interfaces);

  // If no clear mode from start interfaces, check what remains active
  if (new_mode == ControlMode::UNINITIALIZED) {
    // Reset claimed states based on stop interfaces
    for (const auto& interface : stop_interfaces) {
      if (interface.find("/position") != std::string::npos) {
        position_interface_claimed_ = false;
      } else if (interface.find("/velocity") != std::string::npos) {
        velocity_interface_claimed_ = false;
      } else if (interface.find("/effort") != std::string::npos) {
        effort_interface_claimed_ = false;
      }
    }

    // Set claimed states based on start interfaces
    for (const auto& interface : start_interfaces) {
      if (interface.find("/position") != std::string::npos) {
        position_interface_claimed_ = true;
      } else if (interface.find("/velocity") != std::string::npos) {
        velocity_interface_claimed_ = true;
      } else if (interface.find("/effort") != std::string::npos) {
        effort_interface_claimed_ = true;
      }
    }

    // Determine mode based on what's claimed
    if (effort_interface_claimed_) {
      new_mode = ControlMode::MIT;
    } else if (position_interface_claimed_) {
      new_mode = ControlMode::POSITION_VELOCITY;
    }
  }

  pending_mode_ = new_mode;

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10DualModeHW"),
      "Mode switch prepared: %s -> %s",
      current_mode_ == ControlMode::MIT                 ? "MIT"
      : current_mode_ == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY"
                                                        : "UNINITIALIZED",
      pending_mode_ == ControlMode::MIT                 ? "MIT"
      : pending_mode_ == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY"
                                                        : "UNINITIALIZED");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10DualModeHW::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (pending_mode_ == current_mode_) {
    RCLCPP_DEBUG(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                 "No mode change needed");
    return hardware_interface::return_type::OK;
  }

  // Just mark the mode switch as pending
  // The actual switch will happen in write() after sending the last command
  // in the current mode for a smooth transition
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Mode switch from %s to %s will be performed after next write",
              current_mode_ == ControlMode::MIT ? "MIT" :
                (current_mode_ == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY" : "UNINITIALIZED"),
              pending_mode_ == ControlMode::MIT ? "MIT" : "POSITION_VELOCITY");

  // If we're uninitialized, switch immediately
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    bool success = false;
    switch (pending_mode_) {
      case ControlMode::POSITION_VELOCITY:
        success = switch_to_position_mode();
        break;
      case ControlMode::MIT:
        success = switch_to_mit_mode();
        break;
      default:
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                    "Invalid pending mode");
        break;
    }

    if (success) {
      log_mode_switch(current_mode_, pending_mode_);
      current_mode_ = pending_mode_;
      return hardware_interface::return_type::OK;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                   "Failed to switch control mode");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Mode switch is pending and will be executed in write()
  return hardware_interface::return_type::OK;
}

ControlMode OpenArm_v10DualModeHW::determine_mode_from_interfaces(
    const std::vector<std::string>& interfaces) {
  bool has_position = false;
  bool has_effort = false;

  for (const auto& interface : interfaces) {
    if (interface.find("/position") != std::string::npos) {
      has_position = true;
    } else if (interface.find("/effort") != std::string::npos) {
      has_effort = true;
    }
  }

  // Effort interface claimed -> MIT mode for impedance control
  if (has_effort) {
    return ControlMode::MIT;
  }

  // Position interface claimed without effort -> Position-Velocity mode
  if (has_position && !has_effort) {
    return ControlMode::POSITION_VELOCITY;
  }

  return ControlMode::UNINITIALIZED;
}

bool OpenArm_v10DualModeHW::switch_to_position_mode() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Switching motors to Position-Velocity Mode (CTRL_MODE=2)...");

  // Set all motors to position-velocity mode (CTRL_MODE = 2)
  openarm_->write_param_all(
      static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 2);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return true;
}

bool OpenArm_v10DualModeHW::switch_to_mit_mode() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Switching motors to MIT Mode (CTRL_MODE=1)...");

  // Set all motors to MIT mode (CTRL_MODE = 1)
  openarm_->write_param_all(
      static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return true;
}

void OpenArm_v10DualModeHW::log_mode_switch(ControlMode from, ControlMode to) {
  std::string from_str = from == ControlMode::MIT ? "MIT"
                         : from == ControlMode::POSITION_VELOCITY
                             ? "POSITION_VELOCITY"
                             : "UNINITIALIZED";
  std::string to_str = to == ControlMode::MIT ? "MIT"
                       : to == ControlMode::POSITION_VELOCITY
                           ? "POSITION_VELOCITY"
                           : "UNINITIALIZED";

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Control mode switched: %s -> %s", from_str.c_str(),
              to_str.c_str());

  if (to == ControlMode::POSITION_VELOCITY) {
    RCLCPP_INFO(
        rclcpp::get_logger("OpenArm_v10DualModeHW"),
        "Position-Velocity mode active: Using internal motor PID control");
  } else if (to == ControlMode::MIT) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "MIT mode active: Using external impedance control");
  }
}

hardware_interface::CallbackReturn OpenArm_v10DualModeHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Activating OpenArm V10...");

  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Read current motor positions and initialize commands to match
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    pos_commands_[i] = arm_motors[i].get_position();
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Initialized joint %zu command to current position: %.3f rad",
                i, pos_commands_[i]);
  }

  // Initialize gripper command to current position if enabled
  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      double motor_pos = gripper_motors[0].get_position();
      pos_commands_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), motor_pos);
      vel_commands_[gripper_idx] = 0.0;
      tau_commands_[gripper_idx] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "OpenArm V10 activated. Waiting for control mode selection...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10DualModeHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"), "Deactivating...");

  return_to_zero();
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  current_mode_ = ControlMode::UNINITIALIZED;
  position_interface_claimed_ = false;
  velocity_interface_claimed_ = false;
  effort_interface_claimed_ = false;

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10DualModeHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read arm motor states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    const auto& motor = arm_motors[i];

    // Check for unrecoverable motor errors
    if (motor.has_unrecoverable_error()) {
      uint8_t error_code = motor.get_state_error();
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                   "Motor %zu has error: 0x%X", i, error_code);
      return hardware_interface::return_type::ERROR;
    }

    pos_states_[i] = motor.get_position();
    vel_states_[i] = motor.get_velocity();
    tau_states_[i] = motor.get_torque();
  }

  // Read gripper state if enabled
  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();
    const auto& gripper_motors = openarm_->get_gripper().get_motors();

    if (!gripper_motors.empty()) {
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), motor_pos);
      vel_states_[gripper_idx] = 0;  // TODO: implement velocity mapping
      tau_states_[gripper_idx] = 0;  // TODO: implement torque mapping
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10DualModeHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Skip write if mode not initialized
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    return hardware_interface::return_type::OK;
  }

  // Send commands based on current mode
  bool success = false;

  switch (current_mode_) {
    case ControlMode::POSITION_VELOCITY:
      success = send_position_commands();
      break;
    case ControlMode::MIT:
      success = send_mit_commands();
      break;
    default:
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                           *rclcpp::Clock::make_shared(), 1000,
                           "Invalid control mode");
      return hardware_interface::return_type::OK;
  }

  if (!success) {
    return hardware_interface::return_type::ERROR;
  }

  openarm_->recv_all(1000);

  // After sending the command, check if we need to switch modes
  // This ensures smooth transition by sending one last command in the old mode
  // before switching to the new mode
  if (pending_mode_ != current_mode_ && pending_mode_ != ControlMode::UNINITIALIZED) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Performing delayed mode switch from %s to %s after command",
                current_mode_ == ControlMode::MIT ? "MIT" : "POSITION_VELOCITY",
                pending_mode_ == ControlMode::MIT ? "MIT" : "POSITION_VELOCITY");

    bool switch_success = false;
    switch (pending_mode_) {
      case ControlMode::POSITION_VELOCITY:
        switch_success = switch_to_position_mode();
        break;
      case ControlMode::MIT:
        switch_success = switch_to_mit_mode();
        break;
      default:
        break;
    }

    if (switch_success) {
      log_mode_switch(current_mode_, pending_mode_);
      current_mode_ = pending_mode_;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                   "Failed to switch control mode after command");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

bool OpenArm_v10DualModeHW::send_position_commands() {
  // Prepare position-velocity parameters for arm motors
  std::vector<openarm::damiao_motor::PosVelParam> arm_params;

  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    const auto& motor = config_.arm_joints[i];
    double position = pos_commands_[i];

    // Use velocity command if velocity interface is claimed, otherwise use
    // max_velocity
    double velocity;
    if (velocity_interface_claimed_) {
      velocity = vel_commands_[i];
      // Limit velocity to max configured velocity
      if (std::abs(velocity) > motor.max_velocity) {
        velocity = std::copysign(motor.max_velocity, velocity);
      }
    } else {
      // Default to max velocity when only position interface is claimed
      velocity = motor.max_velocity;
    }

    arm_params.push_back({position, velocity});
  }

  // Send all arm position-velocity commands at once
  openarm_->get_arm().pos_vel_control_all(arm_params);

  // Control gripper if enabled
  if (config_.gripper_joint.has_value()) {
    const auto& gripper = config_.gripper_joint.value();
    size_t idx = config_.arm_joints.size();
    double motor_position =
        gripper_joint_to_motor_radians(gripper, pos_commands_[idx]);

    // Use velocity command if velocity interface is claimed, otherwise use
    // max_velocity
    double motor_velocity;
    if (velocity_interface_claimed_) {
      motor_velocity = vel_commands_[idx];
      if (std::abs(motor_velocity) > gripper.max_velocity) {
        motor_velocity = std::copysign(gripper.max_velocity, motor_velocity);
      }
    } else {
      // Default to max velocity when only position interface is claimed
      motor_velocity = gripper.max_velocity;
    }

    openarm_->get_gripper().pos_vel_control_all(
        {{motor_position, motor_velocity}});
  }

  return true;
}

bool OpenArm_v10DualModeHW::send_mit_commands() {
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    const MotorConfig& c = config_.arm_joints[i];
    arm_params.push_back(
        {c.kp, c.kd, pos_commands_[i], vel_commands_[i], tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Control gripper if enabled
  if (config_.gripper_joint.has_value()) {
    const auto& gripper = config_.gripper_joint.value();
    size_t idx = config_.arm_joints.size();
    double motor_command =
        gripper_joint_to_motor_radians(gripper, pos_commands_[idx]);

    openarm_->get_gripper().mit_control_all(
        {{gripper.kp, gripper.kd, motor_command, 0, 0}});
  }

  return true;
}

void OpenArm_v10DualModeHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Returning to zero position...");

  if (current_mode_ == ControlMode::POSITION_VELOCITY) {
    // Use position-velocity control for zero
    std::vector<openarm::damiao_motor::PosVelParam> arm_params;
    for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
      arm_params.push_back({0.0, 0.0});
    }
    openarm_->get_arm().pos_vel_control_all(arm_params);

    if (config_.gripper_joint.has_value()) {
      const auto& gripper = config_.gripper_joint.value();
      double motor_zero =
          gripper_joint_to_motor_radians(gripper, gripper.closed_position);
      openarm_->get_gripper().pos_vel_control_all({{motor_zero, 0.0}});
    }
  } else {
    // Use MIT control for zero
    std::vector<openarm::damiao_motor::MITParam> arm_params;
    for (const auto& config : config_.arm_joints) {
      arm_params.push_back({config.kp, config.kd, 0.0, 0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(arm_params);

    if (config_.gripper_joint.has_value()) {
      const auto& gripper = config_.gripper_joint.value();
      openarm_->get_gripper().mit_control_all(
          {{gripper.kp, gripper.kd, gripper.closed_position, 0.0, 0.0}});
    }
  }

  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

// Helper methods implementation (parse_config, generate_joint_names, etc.)
// These are copied from the original implementation with minor modifications

bool OpenArm_v10DualModeHW::parse_config(
    const hardware_interface::HardwareInfo& info) {
  // Get motor config file path
  auto it = info.hardware_parameters.find("motor_config_file");
  if (it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                 "Required parameter 'motor_config_file' not found");
    return false;
  }
  motor_config_file_ = it->second;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Using motor config file: %s", motor_config_file_.c_str());
  return true;
}

bool OpenArm_v10DualModeHW::generate_joint_names() {
  joint_names_.clear();

  // Add arm joint names
  for (const auto& joint : config_.arm_joints) {
    joint_names_.push_back(joint.name);
  }

  // Add gripper joint name if present
  if (config_.gripper_joint.has_value()) {
    joint_names_.push_back(config_.gripper_joint.value().name);
  }

  return !joint_names_.empty();
}

bool OpenArm_v10DualModeHW::load_motor_config_from_yaml(
    const std::string& yaml_file) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file);
    config_ = config.as<ControllerConfig>();

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Loaded configuration with %zu arm joints",
                config_.arm_joints.size());

    if (config_.gripper_joint.has_value()) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                  "Gripper configuration loaded");
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                 "Failed to load config from %s: %s", yaml_file.c_str(),
                 e.what());
    return false;
  }
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10DualModeHW,
                       hardware_interface::SystemInterface)
