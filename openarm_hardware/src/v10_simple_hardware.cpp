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

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "openarm_hardware/hardware_config.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  try {
    // Parse CAN interface settings from hardware parameters
    auto it = info.hardware_parameters.find("can_interface");
    if (it == info.hardware_parameters.end() || it->second.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Required parameter 'can_interface' not provided");
      return false;
    }
    config_.can_iface = it->second;

    it = info.hardware_parameters.find("can_fd");
    if (it != info.hardware_parameters.end()) {
      config_.can_fd = parse_bool_param(it->second);
    } else {
      config_.can_fd = false;  // Default to false if not specified
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "CAN interface: %s, CAN-FD: %s", config_.can_iface.c_str(),
                config_.can_fd ? "enabled" : "disabled");

    // Parse joint-level parameters
    for (const auto& joint : info.joints) {
      const auto& params = joint.parameters;

      // Check if this is a gripper joint
      auto is_gripper_it = params.find("is_gripper");
      bool is_gripper = (is_gripper_it != params.end()) &&
                        parse_bool_param(is_gripper_it->second);

      if (is_gripper) {
        // Parse gripper configuration
        GripperConfig gripper;
        gripper.name = joint.name;

        gripper.send_can_id = std::stoul(params.at("send_can_id"), nullptr, 0);
        gripper.recv_can_id = std::stoul(params.at("recv_can_id"), nullptr, 0);
        gripper.kp = std::stod(params.at("kp"));
        gripper.kd = std::stod(params.at("kd"));
        gripper.closed_position = std::stod(params.at("closed_position"));
        gripper.open_position = std::stod(params.at("open_position"));
        gripper.motor_closed_radians =
            std::stod(params.at("motor_closed_radians"));
        gripper.motor_open_radians = std::stod(params.at("motor_open_radians"));
        gripper.max_velocity = std::stod(params.at("max_velocity"));

        config_.gripper_joint = gripper;

        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "Configured gripper joint: %s", joint.name.c_str());
      } else {
        // Parse arm joint configuration
        MotorConfig motor;
        motor.name = joint.name;

        motor.send_can_id = std::stoul(params.at("send_can_id"), nullptr, 0);
        motor.recv_can_id = std::stoul(params.at("recv_can_id"), nullptr, 0);
        motor.kp = std::stod(params.at("kp"));
        motor.kd = std::stod(params.at("kd"));
        motor.max_velocity = std::stod(params.at("max_velocity"));

        config_.arm_joints.push_back(motor);

        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                    "Configured arm joint: %s", joint.name.c_str());
      }
    }

    if (config_.arm_joints.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "No arm joints configured");
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Configured %zu arm joints and %s gripper",
                config_.arm_joints.size(),
                config_.gripper_joint.has_value() ? "1" : "0");

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to parse configuration: %s", e.what());
    return false;
  }
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parse configuration from hardware_interface info
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Build joint names vector from config
  joint_names_.clear();
  for (const auto& motor : config_.arm_joints) {
    joint_names_.push_back(motor.name);
  }
  if (config_.gripper_joint.has_value()) {
    joint_names_.push_back(config_.gripper_joint->name);
  }

  // Log the joint count
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configured with %zu joints total", joint_names_.size());

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              config_.can_iface.c_str(),
              config_.can_fd ? "enabled" : "disabled");

  try {
    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(
        config_.can_iface, config_.can_fd);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to initialize OpenArm on interface %s: %s",
                 config_.can_iface.c_str(), e.what());
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Possible causes: CAN interface not found, interface down, "
                 "permission denied, or driver not loaded");
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
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Initializing gripper...");
    const auto& gripper = config_.gripper_joint.value();
    openarm_->init_gripper_motor(gripper.motor_type, gripper.send_can_id,
                                 gripper.recv_can_id);
  }

  // Initialize state and command vectors
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Set all motors to MIT mode (CTRL_MODE = 1)
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Setting all motors to MIT mode (CTRL_MODE=1)...");
  openarm_->write_param_all(
      static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10HW::export_state_interfaces() {
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
OpenArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
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

hardware_interface::CallbackReturn OpenArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Activating OpenArm V10...");
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Read current motor positions and initialize commands to match
  // This prevents the arms from moving when the controller starts
  openarm_->refresh_all();
  openarm_->recv_all();

  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    pos_commands_[i] = arm_motors[i].get_position();
    vel_commands_[i] = 0.0;
    tau_commands_[i] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                "Initialized joint %zu command to current position: %.3f rad",
                i, pos_commands_[i]);
  }

  // Initialize gripper command to current position if enabled
  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();

    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      double motor_pos = gripper_motors[0].get_position();
      pos_commands_[gripper_idx] =
          config_.gripper_joint.value().to_joint(motor_pos);
      vel_commands_[gripper_idx] = 0.0;
      tau_commands_[gripper_idx] = 0.0;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "Initialized gripper command to current position: %.3f",
                  pos_commands_[gripper_idx]);
    }
  }

  // Return to zero position
  // Commented out to prevent accidental collisions during startup
  // Motors will remain at their current position when enabled
  // return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();

  assert(arm_motors.size() == config_.arm_joints.size());

  for (size_t i = 0; i < arm_motors.size(); ++i) {
    const auto& motor = arm_motors[i];

    // Check for unrecoverable motor errors
    if (motor.has_unrecoverable_error()) {
      uint8_t error_code = motor.get_state_error();
      std::string error_msg = error_code_to_string(error_code);
      RCLCPP_ERROR(
          rclcpp::get_logger("OpenArm_v10HW"),
          "Arm motor %zu (CAN ID 0x%03X) has unrecoverable error: %s (0x%X). "
          "Stopping controller.",
          i, motor.get_send_can_id(), error_msg.c_str(), error_code);
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

    // Check all gripper motors for errors
    for (size_t i = 0; i < gripper_motors.size(); ++i) {
      const auto& motor = gripper_motors[i];
      if (motor.has_unrecoverable_error()) {
        uint8_t error_code = motor.get_state_error();
        std::string error_msg = error_code_to_string(error_code);
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                     "Gripper motor %zu (CAN ID 0x%03X) has unrecoverable "
                     "error: %s (0x%X). "
                     "Stopping controller.",
                     i, motor.get_send_can_id(), error_msg.c_str(), error_code);
        return hardware_interface::return_type::ERROR;
      }
    }

    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[gripper_idx] =
          config_.gripper_joint.value().to_joint(motor_pos);

      // Pass through velocity and torque directly
      // TODO: These may need scaling based on gripper mechanism
      vel_states_[gripper_idx] = gripper_motors[0].get_velocity();
      tau_states_[gripper_idx] = gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Check arm motors for errors before sending commands
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    const auto& motor = arm_motors[i];
    if (motor.has_unrecoverable_error()) {
      uint8_t error_code = motor.get_state_error();
      std::string error_msg = error_code_to_string(error_code);
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Cannot send commands: Arm motor %zu (CAN ID 0x%03X) has "
                   "unrecoverable error: %s (0x%X).",
                   i, motor.get_send_can_id(), error_msg.c_str(), error_code);
      return hardware_interface::return_type::ERROR;
    }
  }

  // Check gripper motors for errors before sending commands
  if (config_.gripper_joint.has_value()) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    for (size_t i = 0; i < gripper_motors.size(); ++i) {
      const auto& motor = gripper_motors[i];
      if (motor.has_unrecoverable_error()) {
        uint8_t error_code = motor.get_state_error();
        std::string error_msg = error_code_to_string(error_code);
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                     "Cannot send commands: Gripper motor %zu (CAN ID 0x%03X) "
                     "has unrecoverable error: %s (0x%X).",
                     i, motor.get_send_can_id(), error_msg.c_str(), error_code);
        return hardware_interface::return_type::ERROR;
      }
    }
  }

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
    // There should be at least one extra joint if the gripper is enabled.
    assert(joint_names_.size() == 1 + config_.arm_joints.size());
    // TODO the true mappings are unimplemented.
    size_t idx = config_.arm_joints.size();
    double motor_command = gripper.to_radians(pos_commands_[idx]);

    openarm_->get_gripper().mit_control_all(
        {{gripper.kp, gripper.kd, motor_command, 0, 0}});
  }
  openarm_->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Returning to zero position...");

  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (const auto& config : config_.arm_joints) {
    arm_params.push_back({config.kp, config.kd, 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (config_.gripper_joint.has_value()) {
    const auto& gripper = config_.gripper_joint.value();
    openarm_->get_gripper().mit_control_all(
        {{gripper.kp, gripper.kd, gripper.closed_position, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  openarm_->recv_all();
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
