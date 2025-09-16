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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

// YAML conversion implementations
namespace YAML {

openarm::damiao_motor::MotorType parse_motor_type(const std::string& type_str) {
  if (type_str == "DM4310") return openarm::damiao_motor::MotorType::DM4310;
  if (type_str == "DM4340") return openarm::damiao_motor::MotorType::DM4340;
  if (type_str == "DM8009") return openarm::damiao_motor::MotorType::DM8009;
  if (type_str == "DMH6215") return openarm::damiao_motor::MotorType::DMH6215;
  if (type_str == "DM8006") return openarm::damiao_motor::MotorType::DM8006;
  if (type_str == "DM6006") return openarm::damiao_motor::MotorType::DM6006;

  throw std::runtime_error("Unknown motor type: " + type_str);
}

bool convert<openarm_hardware::MotorConfig>::decode(
    const Node& node, openarm_hardware::MotorConfig& config) {
  if (!node.IsMap()) {
    return false;
  }

  try {
    config.name = node["name"].as<std::string>();

    // Parse motor type
    config.type = parse_motor_type(node["type"].as<std::string>());
    config.send_can_id = node["send_can_id"].as<uint32_t>();
    config.recv_can_id = node["recv_can_id"].as<uint32_t>();
    config.kp = node["kp"].as<double>();
    config.kd = node["kd"].as<double>();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to parse motor config: %s", e.what());
    return false;
  }
}

bool convert<openarm_hardware::GripperConfig>::decode(
    const Node& node, openarm_hardware::GripperConfig& config) {
  if (!node.IsMap()) {
    return false;
  }

  try {
    config.name = node["name"].as<std::string>();

    config.motor_type = parse_motor_type(node["motor_type"].as<std::string>());

    config.send_can_id = node["send_can_id"].as<uint32_t>();
    config.recv_can_id = node["recv_can_id"].as<uint32_t>();
    config.kp = node["kp"].as<double>();
    config.kd = node["kd"].as<double>();

    // Position mapping
    config.closed_position = node["closed_position"].as<double>();
    config.open_position = node["open_position"].as<double>();
    config.motor_closed_radians = node["motor_closed_radians"].as<double>();
    config.motor_open_radians = node["motor_open_radians"].as<double>();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to parse gripper config: %s", e.what());
    return false;
  }
}

bool convert<openarm_hardware::ControllerConfig>::decode(
    const Node& node, openarm_hardware::ControllerConfig& config) {
  if (!node.IsMap()) {
    return false;
  }

  try {
    auto logger = rclcpp::get_logger("OpenArm_v10HW");

    // Load CAN interface configuration (required)
    if (!node["can_iface"]) {
      throw std::runtime_error("Missing required field: can_iface");
    }
    config.can_iface = node["can_iface"].as<std::string>();

    if (!node["can_fd"]) {
      throw std::runtime_error("Missing required field: can_fd");
    }
    config.can_fd = node["can_fd"].as<bool>();

    // Load arm configuration (required)
    if (!node["arm"]) {
      throw std::runtime_error("Missing required field: arm");
    }
    const YAML::Node& arm_node = node["arm"];

    if (!arm_node.IsSequence()) {
      RCLCPP_ERROR(logger, "arm configuration must be a sequence");
      return false;
    }

    for (const auto& motor_node : arm_node) {
      openarm_hardware::MotorConfig motor_config =
          motor_node.as<openarm_hardware::MotorConfig>();
      config.arm_joints.push_back(motor_config);
    }

    RCLCPP_INFO(logger, "Loaded %zu arm motor configurations from YAML",
                config.arm_joints.size());

    // Load auxiliary motors configuration (also a list)
    if (node["auxiliary"]) {
      const YAML::Node& aux_node = node["auxiliary"];

      if (!aux_node.IsSequence()) {
        RCLCPP_INFO(logger, "auxiliary configuration must be a sequence");
        return false;
      }

      for (const auto& motor_node : aux_node) {
        openarm_hardware::MotorConfig motor_config =
            motor_node.as<openarm_hardware::MotorConfig>();
        config.arm_joints.push_back(motor_config);
      }

      RCLCPP_INFO(logger, "Loaded %zu auxiliary motor configurations",
                  aux_node.size());
    }

    // Load gripper configuration
    if (node["gripper"]) {
      config.gripper_joint =
          node["gripper"].as<openarm_hardware::GripperConfig>();
      RCLCPP_INFO(logger, "Loaded gripper configuration");
    }

    return true;
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to parse YAML configuration: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to load YAML configuration: %s", e.what());
    return false;
  }
}

}  // namespace YAML

namespace openarm_hardware {

double gripper_joint_to_motor_radians(const GripperConfig& c,
                                      double joint_value) {
  double range = c.open_position - c.closed_position;
  double motor_range = c.motor_open_radians - c.motor_closed_radians;
  return c.motor_closed_radians +
         ((joint_value - c.closed_position) / range) * motor_range;
}

double gripper_motor_radians_to_joint(const GripperConfig& c,
                                      double motor_radians) {
  double range = c.open_position - c.closed_position;
  double motor_range = c.motor_open_radians - c.motor_closed_radians;
  return c.closed_position +
         ((motor_radians - c.motor_closed_radians) / motor_range) * range;
}

OpenArm_v10HW::OpenArm_v10HW() = default;

bool OpenArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse motor config file path if provided
  auto it = info.hardware_parameters.find("motor_config_file");
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "motor_config_file not provided");
    return false;
  }

  motor_config_file_ = it->second;
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Motor config file specified: %s", motor_config_file_.c_str());

  return true;
}

bool OpenArm_v10HW::load_motor_config_from_yaml(const std::string& yaml_file) {
  try {
    YAML::Node yaml_config = YAML::LoadFile(yaml_file);
    config_ = yaml_config.as<ControllerConfig>();

    // Also set CAN interface and FD from the config
    if (!config_.can_iface.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                  "Loaded configuration from %s", yaml_file.c_str());
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Invalid configuration: missing can_iface");
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "Failed to load config from %s: %s", yaml_file.c_str(),
                 e.what());
    return false;
  }
}

bool OpenArm_v10HW::generate_joint_names() {
  joint_names_.clear();

  // Add arm joint names from configs
  if (config_.arm_joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                 "No arm configurations specified");
    return false;
  }

  for (const auto& motor : config_.arm_joints) {
    joint_names_.push_back(motor.name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added arm joint: %s",
                motor.name.c_str());
  }

  // Add gripper joint name if configured
  if (config_.gripper_joint.has_value()) {
    joint_names_.push_back(config_.gripper_joint->name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"), "Added gripper joint: %s",
                config_.gripper_joint->name.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Total %zu joints configured", joint_names_.size());

  return true;
}

hardware_interface::CallbackReturn OpenArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Parse configuration
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
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Configured with %zu joints total", joint_names_.size());

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              config_.can_iface.c_str(),
              config_.can_fd ? "enabled" : "disabled");
  openarm_ = std::make_unique<openarm::can::socket::OpenArm>(config_.can_iface,
                                                             config_.can_fd);

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

  // Initialize state and command vectors based on generated joint count
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

  // Return to zero position
  return_to_zero();

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
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (config_.gripper_joint.has_value()) {
    size_t gripper_idx = config_.arm_joints.size();
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[gripper_idx] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[gripper_idx] = 0;  // gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
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
    double motor_command =
        gripper_joint_to_motor_radians(gripper, pos_commands_[idx]);

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
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
