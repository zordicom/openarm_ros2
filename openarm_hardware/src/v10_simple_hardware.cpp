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
#include "openarm_hardware/config_yaml.hpp"

#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
// Helper function to convert motor error code to human-readable string
std::string error_code_to_string(uint8_t error_code) {
  switch (error_code) {
    case 0x1:
      return "No error";
    case 0x8:
      return "Overvoltage";
    case 0x9:
      return "Undervoltage";
    case 0xA:
      return "Overcurrent";
    case 0xB:
      return "MOS overtemperature";
    case 0xC:
      return "Motor coil overtemperature";
    case 0xD:
      return "Communication loss";
    case 0xE:
      return "Overload";
    default:
      return "Unknown error (0x" + std::to_string(error_code) + ")";
  }
}
}  // namespace

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

  try {
    openarm_ = std::make_unique<openarm::can::socket::OpenArm>(config_.can_iface,
                                                               config_.can_fd);
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

  // Print all RID values at startup for debugging
  print_all_rid_values();

  // Set all motors to MIT mode (CTRL_MODE = 1)
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Setting all motors to MIT mode (CTRL_MODE=1)...");
  openarm_->write_param_all(static_cast<int>(openarm::damiao_motor::RID::CTRL_MODE), 1);
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
      pos_commands_[gripper_idx] = gripper_motor_radians_to_joint(
          config_.gripper_joint.value(), motor_pos);
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
  openarm_->recv_all(2000);  // Wait up to 2ms for all motor responses to reduce stale data

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();

  assert(arm_motors.size() == config_.arm_joints.size());

  for (size_t i = 0; i < arm_motors.size(); ++i) {
    const auto& motor = arm_motors[i];

    // Check for unrecoverable motor errors
    if (motor.has_unrecoverable_error()) {
      uint8_t error_code = motor.get_state_error();
      std::string error_msg = error_code_to_string(error_code);
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
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
                     "Gripper motor %zu (CAN ID 0x%03X) has unrecoverable error: %s (0x%X). "
                     "Stopping controller.",
                     i, motor.get_send_can_id(), error_msg.c_str(), error_code);
        return hardware_interface::return_type::ERROR;
      }
    }

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
  // Check arm motors for errors before sending commands
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < arm_motors.size(); ++i) {
    const auto& motor = arm_motors[i];
    if (motor.has_unrecoverable_error()) {
      uint8_t error_code = motor.get_state_error();
      std::string error_msg = error_code_to_string(error_code);
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10HW"),
                   "Cannot send commands: Arm motor %zu (CAN ID 0x%03X) has unrecoverable error: %s (0x%X).",
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
                     "Cannot send commands: Gripper motor %zu (CAN ID 0x%03X) has unrecoverable error: %s (0x%X).",
                     i, motor.get_send_can_id(), error_msg.c_str(), error_code);
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    const MotorConfig& c = config_.arm_joints[i];

    // Safety clamp torque commands to motor hardware limits
    double tau_clamped = tau_commands_[i];
    const auto& motor_limits = openarm::damiao_motor::MOTOR_LIMIT_PARAMS[static_cast<size_t>(c.type)];
    double tau_limit = motor_limits.tMax;

    if (std::abs(tau_clamped) > tau_limit) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10HW"),
                           steady_clock, 1000,
                           "Joint %s torque command %.3f Nm exceeds motor limit %.1f Nm - clamping!",
                           c.name.c_str(), tau_clamped, tau_limit);
      tau_clamped = std::clamp(tau_clamped, -tau_limit, tau_limit);
    }

    arm_params.push_back(
        {c.kp, c.kd, pos_commands_[i], vel_commands_[i], tau_clamped});
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

void OpenArm_v10HW::print_all_rid_values() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "======================================");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "Querying all RID values at startup...");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "======================================");

  // Set callback mode to PARAM for reading parameters
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::PARAM);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Define RID names and descriptions for better readability
  struct RIDInfo {
    openarm::damiao_motor::RID rid;
    const char* name;
    const char* description;
    bool is_uint32;  // true if value should be interpreted as uint32, false for float
  };

  // List of important RIDs to query (sorted by RID value)
  std::vector<RIDInfo> rid_list = {
    // Voltage and protection (0-3)
    {openarm::damiao_motor::RID::UV_Value, "UV_Value", "Undervoltage threshold (V)", false},
    {openarm::damiao_motor::RID::KT_Value, "KT_Value", "Motor torque constant (Nm/A)", false},
    {openarm::damiao_motor::RID::OT_Value, "OT_Value", "Overtemperature threshold (°C)", false},
    {openarm::damiao_motor::RID::OC_Value, "OC_Value", "Overcurrent threshold (A)", false},

    // Motion limits (4-6)
    {openarm::damiao_motor::RID::ACC, "ACC", "Acceleration (rad/s²)", false},
    {openarm::damiao_motor::RID::DEC, "DEC", "Deceleration (rad/s²)", false},
    {openarm::damiao_motor::RID::MAX_SPD, "MAX_SPD", "Maximum speed (rad/s)", false},

    // CAN IDs and communication (7-10)
    {openarm::damiao_motor::RID::MST_ID, "MST_ID", "Master CAN ID (recv ID)", true},
    {openarm::damiao_motor::RID::ESC_ID, "ESC_ID", "ESC (motor) CAN ID", true},
    {openarm::damiao_motor::RID::TIMEOUT, "TIMEOUT", "Communication timeout (ms)", true},
    {openarm::damiao_motor::RID::CTRL_MODE, "CTRL_MODE", "Control mode (1=MIT, 2=PV)", true},

    // Control parameters (11-12)
    {openarm::damiao_motor::RID::Damp, "Damp", "Damping coefficient", false},
    {openarm::damiao_motor::RID::Inertia, "Inertia", "Moment of inertia (kg·m²)", false},

    // Version info (13-16)
    {openarm::damiao_motor::RID::hw_ver, "hw_ver", "Hardware version", true},
    {openarm::damiao_motor::RID::sw_ver, "sw_ver", "Software version", true},
    {openarm::damiao_motor::RID::SN, "SN", "Serial number", true},
    {openarm::damiao_motor::RID::NPP, "NPP", "Number of pole pairs", true},

    // Motor constants (17-20)
    {openarm::damiao_motor::RID::Rs, "Rs", "Stator resistance (Ohm)", false},
    {openarm::damiao_motor::RID::LS, "LS", "Stator inductance (H)", false},
    {openarm::damiao_motor::RID::Flux, "Flux", "Flux linkage (Wb)", false},
    {openarm::damiao_motor::RID::Gr, "Gr", "Gear ratio", false},

    // Limits (21-23)
    {openarm::damiao_motor::RID::PMAX, "PMAX", "Max position (rad)", false},
    {openarm::damiao_motor::RID::VMAX, "VMAX", "Max velocity (rad/s)", false},
    {openarm::damiao_motor::RID::TMAX, "TMAX", "Max torque (Nm)", false},

    // Control loop gains (24-28)
    {openarm::damiao_motor::RID::I_BW, "I_BW", "Current loop bandwidth (Hz)", false},
    {openarm::damiao_motor::RID::KP_ASR, "KP_ASR", "Speed loop Kp", false},
    {openarm::damiao_motor::RID::KI_ASR, "KI_ASR", "Speed loop Ki", false},
    {openarm::damiao_motor::RID::KP_APR, "KP_APR", "Position loop Kp", false},
    {openarm::damiao_motor::RID::KI_APR, "KI_APR", "Position loop Ki", false},

    // Additional parameters (29-36)
    {openarm::damiao_motor::RID::OV_Value, "OV_Value", "Overvoltage threshold (V)", false},
    {openarm::damiao_motor::RID::V_BW, "V_BW", "Velocity loop bandwidth (Hz)", false},
    {openarm::damiao_motor::RID::can_br, "can_br", "CAN baudrate", true},
    {openarm::damiao_motor::RID::sub_ver, "sub_ver", "Sub version", true},

    // Calibration offsets (50-55)
    {openarm::damiao_motor::RID::u_off, "u_off", "U-phase offset", false},
    {openarm::damiao_motor::RID::v_off, "v_off", "V-phase offset", false},
    {openarm::damiao_motor::RID::m_off, "m_off", "Mechanical offset", false},
    {openarm::damiao_motor::RID::dir, "dir", "Motor direction", false},
  };

  // Get arm motors
  const auto& arm_motors = openarm_->get_arm().get_motors();

  // Query and print RID values for each arm motor
  for (size_t motor_idx = 0; motor_idx < arm_motors.size(); ++motor_idx) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
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
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                    "  RID %3d (%s): <no response> - %s",
                    static_cast<int>(rid_info.rid), rid_info.name, rid_info.description);
      } else {
        if (rid_info.is_uint32) {
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                      "  RID %3d (%s): %u - %s",
                      static_cast<int>(rid_info.rid), rid_info.name,
                      static_cast<uint32_t>(value), rid_info.description);
        } else {
          RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
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
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
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
          RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10HW"),
                      "  RID %3d (%s): <no response> - %s",
                      static_cast<int>(rid_info.rid), rid_info.name, rid_info.description);
        } else {
          if (rid_info.is_uint32) {
            RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                        "  RID %3d (%s): %u - %s",
                        static_cast<int>(rid_info.rid), rid_info.name,
                        static_cast<uint32_t>(value), rid_info.description);
          } else {
            RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
                        "  RID %3d (%s): %.6f - %s",
                        static_cast<int>(rid_info.rid), rid_info.name,
                        value, rid_info.description);
          }
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "======================================");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "RID value query complete!");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10HW"),
              "======================================");

  // Reset callback mode back to IGNORE after parameter queries
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10HW,
                       hardware_interface::SystemInterface)
