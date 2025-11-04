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

  // Print all RID values at startup for debugging
  print_all_rid_values();

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
  RCLCPP_DEBUG(rclcpp::get_logger("OpenArm_v10DualModeHW"),
               "Preparing command mode switch (%zu starting, %zu stopping)",
               start_interfaces.size(), stop_interfaces.size());

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

  if (pending_mode_ != current_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "Mode switch prepared: %s -> %s",
                current_mode_ == ControlMode::MIT ? "MIT" :
                current_mode_ == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY" : "UNINITIALIZED",
                pending_mode_ == ControlMode::MIT ? "MIT" : "POSITION_VELOCITY");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10DualModeHW::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (pending_mode_ == current_mode_) {
    return hardware_interface::return_type::OK;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("OpenArm_v10DualModeHW"),
               "Mode switch will be performed after next write cycle");

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
              "Switching to Position-Velocity mode");

  // Set callback mode to PARAM for parameter write
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

  // Write CTRL_MODE parameter and wait for response
  openarm_->write_param_all(
      static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 2);
  openarm_->recv_all(5000);  // Wait up to 5ms for parameter write responses

  // Set callback mode back to STATE for normal operation
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

  return true;
}

bool OpenArm_v10DualModeHW::switch_to_mit_mode() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "=== Beginning transition to MIT mode ===");

  // Step 1: Capture current state before transition
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  std::vector<double> pre_switch_pos(arm_motors.size());
  std::vector<double> pre_switch_vel(arm_motors.size());
  std::vector<double> pre_switch_tau(arm_motors.size());

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 1: Current state in Position-Velocity mode:");
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    pre_switch_pos[i] = arm_motors[i].get_position();
    pre_switch_vel[i] = arm_motors[i].get_velocity();
    pre_switch_tau[i] = arm_motors[i].get_torque();

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "  Joint %zu: pos=%.4f rad, vel=%.4f rad/s, tau=%.4f Nm",
                i, pre_switch_pos[i], pre_switch_vel[i], pre_switch_tau[i]);
  }

  // Step 2: Seed command buffers with current state
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    pos_commands_[i] = pre_switch_pos[i];
    vel_commands_[i] = pre_switch_vel[i];
    tau_commands_[i] = pre_switch_tau[i];
  }

  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      pos_commands_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), gripper_motors[0].get_position());
      vel_commands_[gripper_idx] = gripper_motors[0].get_velocity();
      tau_commands_[gripper_idx] = gripper_motors[0].get_torque();
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 2: Command buffers seeded with current state");

  // Step 3: Send MIT command BEFORE changing mode (while still in POS_VEL mode)
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 3: Sending MIT command (while still in Position-Velocity mode):");

  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    const MotorConfig& c = config_.arm_joints[i];
    arm_params.push_back({c.kp, c.kd, pos_commands_[i], vel_commands_[i], tau_commands_[i]});

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "  Joint %zu MIT params: kp=%.1f, kd=%.1f, pos=%.4f, vel=%.4f, tau=%.4f",
                i, c.kp, c.kd, pos_commands_[i], vel_commands_[i], tau_commands_[i]);
  }

  openarm_->get_arm().mit_control_all(arm_params);
  openarm_->recv_all(1000);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 4: MIT command sent, now changing CTRL_MODE to 1");

  // Step 4: Set callback mode to PARAM for parameter write
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);

  // Write CTRL_MODE parameter and wait for response
  openarm_->write_param_all(
      static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 1);
  openarm_->recv_all(5000);  // Wait up to 5ms for parameter write responses

  // Set callback mode back to STATE for normal operation
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

  // Step 5: Read first feedback after mode change
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 5: Reading first feedback in MIT mode");

  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors_post = openarm_->get_arm().get_motors();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Step 6: Validating transition (checking for deviations):");

  bool transition_ok = true;
  const double pos_threshold = 0.01;  // 0.01 rad (~0.57 degrees)
  const double tau_threshold = 0.5;   // 0.5 Nm

  for (size_t i = 0; i < arm_motors_post.size(); ++i) {
    double post_pos = arm_motors_post[i].get_position();
    double post_vel = arm_motors_post[i].get_velocity();
    double post_tau = arm_motors_post[i].get_torque();

    double pos_delta = std::abs(post_pos - pre_switch_pos[i]);
    double tau_delta = std::abs(post_tau - pre_switch_tau[i]);

    const char* status = (pos_delta < pos_threshold && tau_delta < tau_threshold) ? "OK" : "WARN";

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "  Joint %zu [%s]: pos=%.4f (Δ=%.4f), vel=%.4f, tau=%.4f (Δ=%.4f)",
                i, status, post_pos, pos_delta, post_vel, post_tau, tau_delta);

    if (pos_delta >= pos_threshold || tau_delta >= tau_threshold) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                  "  Joint %zu exceeded threshold! pos_delta=%.4f (thresh=%.4f), tau_delta=%.4f (thresh=%.4f)",
                  i, pos_delta, pos_threshold, tau_delta, tau_threshold);
      transition_ok = false;
    }
  }

  if (transition_ok) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "=== Transition to MIT mode completed successfully ===");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "=== Transition to MIT mode completed with deviations (see above) ===");
  }

  return true;
}

void OpenArm_v10DualModeHW::log_mode_switch(ControlMode from, ControlMode to) {
  const char* from_str = from == ControlMode::MIT ? "MIT" :
                         from == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY" : "UNINITIALIZED";
  const char* to_str = to == ControlMode::MIT ? "MIT" :
                       to == ControlMode::POSITION_VELOCITY ? "POSITION_VELOCITY" : "UNINITIALIZED";

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Mode switched: %s -> %s", from_str, to_str);
}

hardware_interface::CallbackReturn OpenArm_v10DualModeHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Activating OpenArm V10...");

  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Initialize commands from current motor state
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    pos_commands_[i] = arm_motors[i].get_position();
    vel_commands_[i] = arm_motors[i].get_velocity();
    tau_commands_[i] = arm_motors[i].get_torque();
  }

  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      pos_commands_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), gripper_motors[0].get_position());
      vel_commands_[gripper_idx] = gripper_motors[0].get_velocity();
      tau_commands_[gripper_idx] = gripper_motors[0].get_torque();
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Hardware activated, commands initialized from current state");

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
  // Request fresh motor states
  openarm_->refresh_all();
  openarm_->recv_all(2000);  // Wait up to 2ms for all motor responses to reduce stale data

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
      pos_states_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), gripper_motors[0].get_position());
      vel_states_[gripper_idx] = 0;
      tau_states_[gripper_idx] = 0;
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

  // Perform delayed mode switch after final command in current mode
  if (pending_mode_ != current_mode_ && pending_mode_ != ControlMode::UNINITIALIZED) {

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

bool OpenArm_v10DualModeHW::parse_config(
    const hardware_interface::HardwareInfo& info) {
  auto it = info.hardware_parameters.find("motor_config_file");
  if (it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                 "Required parameter 'motor_config_file' not found");
    return false;
  }
  motor_config_file_ = it->second;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Motor config file: %s", motor_config_file_.c_str());
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

void OpenArm_v10DualModeHW::print_all_rid_values() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "======================================");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "Querying all RID values at startup...");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "======================================");

  // Define RID names and descriptions for better readability
  struct RIDInfo {
    openarm::damiao_motor::RID rid;
    const char* name;
    const char* description;
    bool is_uint32;  // true if value should be interpreted as uint32, false for float
  };

  // List of important RIDs to query
  std::vector<RIDInfo> rid_list = {
    // Voltage and protection
    {openarm::damiao_motor::RID::UV_Value, "UV_Value", "Undervoltage threshold (V)", false},
    {openarm::damiao_motor::RID::OV_Value, "OV_Value", "Overvoltage threshold (V)", false},
    {openarm::damiao_motor::RID::OT_Value, "OT_Value", "Overtemperature threshold (°C)", false},
    {openarm::damiao_motor::RID::OC_Value, "OC_Value", "Overcurrent threshold (A)", false},

    // Motor constants
    {openarm::damiao_motor::RID::KT_Value, "KT_Value", "Motor torque constant (Nm/A)", false},
    {openarm::damiao_motor::RID::NPP, "NPP", "Number of pole pairs", true},
    {openarm::damiao_motor::RID::Rs, "Rs", "Stator resistance (Ohm)", false},
    {openarm::damiao_motor::RID::LS, "LS", "Stator inductance (H)", false},
    {openarm::damiao_motor::RID::Flux, "Flux", "Flux linkage (Wb)", false},
    {openarm::damiao_motor::RID::Gr, "Gr", "Gear ratio", false},

    // Motion limits
    {openarm::damiao_motor::RID::ACC, "ACC", "Acceleration (rad/s²)", false},
    {openarm::damiao_motor::RID::DEC, "DEC", "Deceleration (rad/s²)", false},
    {openarm::damiao_motor::RID::MAX_SPD, "MAX_SPD", "Maximum speed (rad/s)", false},
    {openarm::damiao_motor::RID::PMAX, "PMAX", "Max position (rad)", false},
    {openarm::damiao_motor::RID::VMAX, "VMAX", "Max velocity (rad/s)", false},
    {openarm::damiao_motor::RID::TMAX, "TMAX", "Max torque (Nm)", false},

    // CAN IDs and communication
    {openarm::damiao_motor::RID::MST_ID, "MST_ID", "Master CAN ID (recv ID)", true},
    {openarm::damiao_motor::RID::ESC_ID, "ESC_ID", "ESC (motor) CAN ID", true},
    {openarm::damiao_motor::RID::TIMEOUT, "TIMEOUT", "Communication timeout (ms)", true},
    {openarm::damiao_motor::RID::can_br, "can_br", "CAN baudrate", true},

    // Control mode and parameters
    {openarm::damiao_motor::RID::CTRL_MODE, "CTRL_MODE", "Control mode (1=MIT, 2=PV)", true},
    {openarm::damiao_motor::RID::Damp, "Damp", "Damping coefficient", false},
    {openarm::damiao_motor::RID::Inertia, "Inertia", "Moment of inertia (kg·m²)", false},

    // Control loop gains
    {openarm::damiao_motor::RID::I_BW, "I_BW", "Current loop bandwidth (Hz)", false},
    {openarm::damiao_motor::RID::V_BW, "V_BW", "Velocity loop bandwidth (Hz)", false},
    {openarm::damiao_motor::RID::KP_ASR, "KP_ASR", "Speed loop Kp", false},
    {openarm::damiao_motor::RID::KI_ASR, "KI_ASR", "Speed loop Ki", false},
    {openarm::damiao_motor::RID::KP_APR, "KP_APR", "Position loop Kp", false},
    {openarm::damiao_motor::RID::KI_APR, "KI_APR", "Position loop Ki", false},

    // Version info
    {openarm::damiao_motor::RID::hw_ver, "hw_ver", "Hardware version", true},
    {openarm::damiao_motor::RID::sw_ver, "sw_ver", "Software version", true},
    {openarm::damiao_motor::RID::sub_ver, "sub_ver", "Sub version", true},
    {openarm::damiao_motor::RID::SN, "SN", "Serial number", true},

    // Calibration offsets
    {openarm::damiao_motor::RID::u_off, "u_off", "U-phase offset", false},
    {openarm::damiao_motor::RID::v_off, "v_off", "V-phase offset", false},
    {openarm::damiao_motor::RID::m_off, "m_off", "Mechanical offset", false},
    {openarm::damiao_motor::RID::dir, "dir", "Motor direction", false},
  };

  // Get arm motors
  const auto& arm_motors = openarm_->get_arm().get_motors();

  // Query and print RID values for each arm motor
  for (size_t motor_idx = 0; motor_idx < arm_motors.size(); ++motor_idx) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                "\n--- Motor %zu: %s (send_id: 0x%02X, recv_id: 0x%02X) ---",
                motor_idx, config_.arm_joints[motor_idx].name.c_str(),
                config_.arm_joints[motor_idx].send_can_id,
                config_.arm_joints[motor_idx].recv_can_id);

    // Query each RID
    for (const auto& rid_info : rid_list) {
      openarm_->query_param_all(static_cast<int>(rid_info.rid));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      openarm_->recv_all();

      // Get the value from the motor
      double value = arm_motors[motor_idx].get_param(static_cast<int>(rid_info.rid));

      // Format and print the value
      if (value == -1) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                    "  RID %3d (%s): <no response> - %s",
                    static_cast<int>(rid_info.rid), rid_info.name, rid_info.description);
      } else {
        if (rid_info.is_uint32) {
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                      "  RID %3d (%s): %u - %s",
                      static_cast<int>(rid_info.rid), rid_info.name,
                      static_cast<uint32_t>(value), rid_info.description);
        } else {
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                      "  RID %3d (%s): %.6f - %s",
                      static_cast<int>(rid_info.rid), rid_info.name,
                      value, rid_info.description);
        }
      }
    }
  }

  // Query and print RID values for gripper if present
  if (config_.gripper_joint.has_value()) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                  "\n--- Gripper Motor: %s (send_id: 0x%02X, recv_id: 0x%02X) ---",
                  config_.gripper_joint.value().name.c_str(),
                  config_.gripper_joint.value().send_can_id,
                  config_.gripper_joint.value().recv_can_id);

      for (const auto& rid_info : rid_list) {
        openarm_->query_param_all(static_cast<int>(rid_info.rid));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        openarm_->recv_all();

        double value = gripper_motors[0].get_param(static_cast<int>(rid_info.rid));

        if (value == -1) {
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                      "  RID %3d (%s): <no response> - %s",
                      static_cast<int>(rid_info.rid), rid_info.name, rid_info.description);
        } else {
          if (rid_info.is_uint32) {
            RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                        "  RID %3d (%s): %u - %s",
                        static_cast<int>(rid_info.rid), rid_info.name,
                        static_cast<uint32_t>(value), rid_info.description);
          } else {
            RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
                        "  RID %3d (%s): %.6f - %s",
                        static_cast<int>(rid_info.rid), rid_info.name,
                        value, rid_info.description);
          }
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "======================================");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "RID value query complete!");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10DualModeHW"),
              "======================================");
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10DualModeHW,
                       hardware_interface::SystemInterface)
