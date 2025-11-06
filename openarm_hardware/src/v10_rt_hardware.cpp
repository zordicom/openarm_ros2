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

#include <pthread.h>
#include <sched.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>

#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

// RT-safe logging - just skip the throttling in RT context for simplicity
static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

namespace openarm_hardware {

// Control mode enum (matches header definition)
using ControlMode = OpenArm_v10RTHardware::ControlMode;

OpenArm_v10RTHardware::OpenArm_v10RTHardware() {
  // Pre-allocate command and state buffers
  command_buffer_.writeFromNonRT(CommandData{});
  state_buffer_.writeFromNonRT(StateData{});
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

  // Mode switching will be handled by worker thread

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Successfully initialized RT hardware interface with %zu joints",
              num_joints_);

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Load motor configuration
  if (!motor_config_file_.empty()) {
    if (!load_motor_config_from_yaml(motor_config_file_)) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to load motor configuration from %s",
                   motor_config_file_.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Create RT-safe OpenArm interface
  openarm_rt_ = std::make_unique<openarm::can::RTSafeOpenArm>();

  if (!openarm_rt_->init(config_.can_interface)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to initialize RT-safe OpenArm interface");
    return CallbackReturn::ERROR;
  }

  // Add motors to RT-safe wrapper based on configuration
  for (const auto& motor : motor_configs_) {
    int motor_idx = openarm_rt_->add_motor(motor.type, motor.send_can_id,
                                           motor.recv_can_id);
    if (motor_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to add motor '%s' to RT-safe wrapper",
                   motor.name.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Added motor '%s' at index %d", motor.name.c_str(), motor_idx);
  }

  // Add gripper motor if configured
  if (gripper_config_.has_value()) {
    int gripper_idx = openarm_rt_->add_motor(gripper_config_->motor_type,
                                             gripper_config_->send_can_id,
                                             gripper_config_->recv_can_id);
    if (gripper_idx < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to add gripper motor to RT-safe wrapper");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Added gripper motor at index %d", gripper_idx);
  }

  // Set initial control mode to UNINITIALIZED until motors are enabled
  current_mode_ = ControlMode::UNINITIALIZED;

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface configured successfully");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Note: The main RT thread priority is set by the controller manager,
  // not by the hardware interface. We only set priority for our worker thread.

  // Check if RT kernel is available
  if (!realtime_tools::has_realtime_kernel()) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "RT kernel not detected, running without RT guarantees");
  }

  // Start CAN worker thread
  worker_running_ = true;
  can_worker_thread_ =
      std::thread(&OpenArm_v10RTHardware::can_worker_loop, this);

  // Configure worker thread (non-RT, lower priority)
  if (config_.worker_thread_priority > 0) {
    struct sched_param param;
    param.sched_priority = config_.worker_thread_priority;

    if (pthread_setschedparam(can_worker_thread_.native_handle(), SCHED_FIFO,
                              &param) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Failed to set worker thread RT priority to %d",
                  config_.worker_thread_priority);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Set worker thread priority to %d (SCHED_FIFO)",
                  config_.worker_thread_priority);
    }
  }

  // Set CPU affinity if configured
  if (!config_.cpu_affinity.empty()) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (int cpu : config_.cpu_affinity) {
      CPU_SET(cpu, &cpuset);
    }

    if (pthread_setaffinity_np(can_worker_thread_.native_handle(),
                               sizeof(cpu_set_t), &cpuset) != 0) {
      RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Failed to set CPU affinity for CAN worker thread");
    } else {
      std::stringstream ss;
      for (size_t i = 0; i < config_.cpu_affinity.size(); ++i) {
        if (i > 0) ss << ",";
        ss << config_.cpu_affinity[i];
      }
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Set worker thread CPU affinity to cores: %s",
                  ss.str().c_str());
    }
  }

  // Enable motors on activation so they can respond to commands
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Enabling motors on activation");

  size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
  if (enabled != openarm_rt_->get_motor_count()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to enable all motors: %zu/%zu", enabled,
                 openarm_rt_->get_motor_count());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
      rclcpp::get_logger("OpenArm_v10RTHardware"),
      "Hardware interface activated successfully. Motors enabled and ready.");

  return CallbackReturn::SUCCESS;
}

OpenArm_v10RTHardware::CallbackReturn OpenArm_v10RTHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Stop worker thread
  worker_running_ = false;
  if (can_worker_thread_.joinable()) {
    can_worker_thread_.join();
  }

  // Disable motors
  if (openarm_rt_ && openarm_rt_->is_ready()) {
    openarm_rt_->disable_all_motors_rt(1000);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface deactivated");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10RTHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
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

  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_commands_[i]);
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type OpenArm_v10RTHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Measure actual function execution time
  auto function_start = std::chrono::steady_clock::now();

  // This is called from RT context - just copy from the realtime buffer
  auto state = state_buffer_.readFromRT();

  if (state && state->valid) {
    for (size_t i = 0; i < num_joints_; ++i) {
      pos_states_[i] = state->positions[i];
      vel_states_[i] = state->velocities[i];
      tau_states_[i] = state->torques[i];

      // Check for motor errors (RT-safe)
      // Only the upper nibble (bits 4-7) indicates actual errors
      // Lower nibble (bits 0-3) is status information
      if ((state->error_codes[i] & 0xF0) != 0) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArm_v10RTHardware"),
                              steady_clock, LOG_THROTTLE_MS,
                              "Motor %zu error code: 0x%02X", i,
                              state->error_codes[i]);
      }
    }
  }

  // Monitor actual function execution time (RT-safe)
  auto function_end = std::chrono::steady_clock::now();
  auto read_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                           function_end - function_start)
                           .count();

  // Only warn if the actual function takes too long (should be < 100us
  // typically)
  if (read_duration > 100) {
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("OpenArm_v10RTHardware"), steady_clock,
        LOG_THROTTLE_MS,
        "Read function execution time: %ld us (expected < 100 us)",
        read_duration);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10RTHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Measure actual function execution time
  auto function_start = std::chrono::steady_clock::now();

  // This is called from RT context - just write to the realtime buffer
  CommandData cmd;
  cmd.mode = current_mode_.load();
  cmd.valid = true;

  for (size_t i = 0; i < num_joints_; ++i) {
    cmd.positions[i] = pos_commands_[i];
    cmd.velocities[i] = vel_commands_[i];
    cmd.torques[i] = tau_commands_[i];
  }

  command_buffer_.writeFromNonRT(cmd);

  // Monitor actual function execution time (RT-safe)
  auto function_end = std::chrono::steady_clock::now();
  auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            function_end - function_start)
                            .count();

  // Only warn if the actual function takes too long (should be < 100us
  // typically)
  if (write_duration > 100) {
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("OpenArm_v10RTHardware"), steady_clock,
        LOG_THROTTLE_MS,
        "Write function execution time: %ld us (expected < 100 us)",
        write_duration);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10RTHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "prepare_command_mode_switch called with %zu start interfaces "
              "and %zu stop interfaces",
              start_interfaces.size(), stop_interfaces.size());

  // Log the interfaces being started
  for (const auto& interface : start_interfaces) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "  Starting interface: %s", interface.c_str());
  }

  // Determine new mode from interfaces
  ControlMode new_mode = determine_mode_from_interfaces(start_interfaces);

  if (new_mode == ControlMode::UNINITIALIZED) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Invalid interface combination for mode switch");
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Determined new mode: %d", static_cast<int>(new_mode));

  // Store pending mode
  pending_mode_ = new_mode;

  // Update interface claiming states based on stop_interfaces
  for (const auto& interface : stop_interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
      position_interface_claimed_ = false;
    } else if (interface.find(hardware_interface::HW_IF_VELOCITY) !=
               std::string::npos) {
      velocity_interface_claimed_ = false;
    } else if (interface.find(hardware_interface::HW_IF_EFFORT) !=
               std::string::npos) {
      effort_interface_claimed_ = false;
    }
  }

  // Update interface claiming states based on start_interfaces
  for (const auto& interface : start_interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
      position_interface_claimed_ = true;
    } else if (interface.find(hardware_interface::HW_IF_VELOCITY) !=
               std::string::npos) {
      velocity_interface_claimed_ = true;
    } else if (interface.find(hardware_interface::HW_IF_EFFORT) !=
               std::string::npos) {
      effort_interface_claimed_ = true;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10RTHardware::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  // Request mode switch (non-blocking for RT safety)
  mode_switch_requested_ = true;

  return hardware_interface::return_type::OK;
}

bool OpenArm_v10RTHardware::parse_config(
    const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface
  auto it = info.hardware_parameters.find("can_interface");
  if (it != info.hardware_parameters.end()) {
    config_.can_interface = it->second;
  }

  // Parse motor config file
  it = info.hardware_parameters.find("motor_config_file");
  if (it != info.hardware_parameters.end()) {
    motor_config_file_ = it->second;
  }

  // Parse RT parameters
  it = info.hardware_parameters.find("rt_priority");
  if (it != info.hardware_parameters.end()) {
    config_.rt_priority = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("worker_thread_priority");
  if (it != info.hardware_parameters.end()) {
    config_.worker_thread_priority = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("cpu_affinity");
  if (it != info.hardware_parameters.end()) {
    std::stringstream ss(it->second);
    std::string cpu;
    while (std::getline(ss, cpu, ',')) {
      config_.cpu_affinity.push_back(std::stoi(cpu));
    }
  }

  it = info.hardware_parameters.find("can_timeout_us");
  if (it != info.hardware_parameters.end()) {
    config_.can_timeout_us = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("max_cycle_time_us");
  if (it != info.hardware_parameters.end()) {
    config_.max_cycle_time_us = std::stoi(it->second);
  }

  // Get joint names and count
  joint_names_.clear();
  for (const auto& joint : info.joints) {
    joint_names_.push_back(joint.name);
  }
  num_joints_ = joint_names_.size();

  return true;
}

bool OpenArm_v10RTHardware::load_motor_config_from_yaml(
    const std::string& file_path) {
  auto logger = rclcpp::get_logger("OpenArm_v10RTHardware");

  try {
    RCLCPP_INFO(logger, "Loading motor configuration from %s",
                file_path.c_str());

    // Load YAML file
    YAML::Node config = YAML::LoadFile(file_path);

    // Parse global settings if present
    if (config["can_interface"]) {
      config_.can_interface = config["can_interface"].as<std::string>();
    }
    if (config["can_timeout_us"]) {
      config_.can_timeout_us = config["can_timeout_us"].as<int>();
    }
    if (config["control_frequency"]) {
      // Just for information, actual frequency is set by controller manager
      int freq = config["control_frequency"].as<int>();
      RCLCPP_INFO(logger, "Control frequency from config: %d Hz", freq);
    }

    // Clear previous configurations
    motor_configs_.clear();
    gripper_config_.reset();

    // Parse motor configurations
    if (!config["motors"]) {
      RCLCPP_ERROR(logger, "No 'motors' section found in configuration file");
      return false;
    }

    const YAML::Node& motors = config["motors"];
    for (YAML::const_iterator it = motors.begin(); it != motors.end(); ++it) {
      std::string joint_name = it->first.as<std::string>();
      const YAML::Node& motor = it->second;

      MotorConfig motor_config;
      motor_config.name = joint_name;

      // Parse motor type (required)
      if (!motor["motor_type"]) {
        RCLCPP_ERROR(logger, "Missing 'motor_type' for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      std::string motor_type_str = motor["motor_type"].as<std::string>();
      if (motor_type_str == "DM3507") {
        motor_config.type = openarm::damiao_motor::MotorType::DM3507;
      } else if (motor_type_str == "DM4310") {
        motor_config.type = openarm::damiao_motor::MotorType::DM4310;
      } else if (motor_type_str == "DM4310_48V") {
        motor_config.type = openarm::damiao_motor::MotorType::DM4310_48V;
      } else if (motor_type_str == "DM4340") {
        motor_config.type = openarm::damiao_motor::MotorType::DM4340;
      } else if (motor_type_str == "DM4340_48V") {
        motor_config.type = openarm::damiao_motor::MotorType::DM4340_48V;
      } else if (motor_type_str == "DM6006") {
        motor_config.type = openarm::damiao_motor::MotorType::DM6006;
      } else if (motor_type_str == "DM8006") {
        motor_config.type = openarm::damiao_motor::MotorType::DM8006;
      } else if (motor_type_str == "DM8009") {
        motor_config.type = openarm::damiao_motor::MotorType::DM8009;
      } else if (motor_type_str == "DM10010L") {
        motor_config.type = openarm::damiao_motor::MotorType::DM10010L;
      } else if (motor_type_str == "DM10010") {
        motor_config.type = openarm::damiao_motor::MotorType::DM10010;
      } else {
        RCLCPP_ERROR(logger, "Unknown motor type: %s", motor_type_str.c_str());
        return false;
      }

      // Parse CAN IDs (required)
      if (!motor["send_can_id"]) {
        RCLCPP_ERROR(logger, "Missing 'send_can_id' for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      if (!motor["recv_can_id"]) {
        RCLCPP_ERROR(logger, "Missing 'recv_can_id' for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      motor_config.send_can_id = motor["send_can_id"].as<uint32_t>();
      motor_config.recv_can_id = motor["recv_can_id"].as<uint32_t>();

      // Parse MIT mode parameters (required)
      if (!motor["mit_mode"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode' section for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      if (!motor["mit_mode"]["kp"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode.kp' for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      if (!motor["mit_mode"]["kd"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode.kd' for motor '%s'",
                     joint_name.c_str());
        return false;
      }
      motor_config.kp = motor["mit_mode"]["kp"].as<double>();
      motor_config.kd = motor["mit_mode"]["kd"].as<double>();

      motor_configs_.push_back(motor_config);
      RCLCPP_INFO(logger,
                  "Configured motor '%s': type=%s, send_id=0x%X, recv_id=0x%X, "
                  "kp=%.2f, kd=%.2f",
                  joint_name.c_str(), motor_type_str.c_str(),
                  motor_config.send_can_id, motor_config.recv_can_id,
                  motor_config.kp, motor_config.kd);
    }

    // Parse optional gripper configuration
    if (config["gripper"]) {
      const YAML::Node& gripper = config["gripper"];
      GripperConfig gripper_cfg;

      gripper_cfg.name = "gripper";

      // Parse motor type (required)
      if (!gripper["motor_type"]) {
        RCLCPP_ERROR(logger, "Missing 'motor_type' for gripper");
        return false;
      }
      std::string motor_type_str = gripper["motor_type"].as<std::string>();

      // Parse motor type using same logic as motors
      if (motor_type_str == "DM3507") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM3507;
      } else if (motor_type_str == "DM4310") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM4310;
      } else if (motor_type_str == "DM4310_48V") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM4310_48V;
      } else if (motor_type_str == "DM4340") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM4340;
      } else if (motor_type_str == "DM4340_48V") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM4340_48V;
      } else if (motor_type_str == "DM6006") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM6006;
      } else if (motor_type_str == "DM8006") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM8006;
      } else if (motor_type_str == "DM8009") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM8009;
      } else if (motor_type_str == "DM10010L") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM10010L;
      } else if (motor_type_str == "DM10010") {
        gripper_cfg.motor_type = openarm::damiao_motor::MotorType::DM10010;
      } else {
        RCLCPP_ERROR(logger, "Unknown motor type for gripper: %s",
                     motor_type_str.c_str());
        return false;
      }

      // Parse CAN IDs (required)
      if (!gripper["send_can_id"]) {
        RCLCPP_ERROR(logger, "Missing 'send_can_id' for gripper");
        return false;
      }
      if (!gripper["recv_can_id"]) {
        RCLCPP_ERROR(logger, "Missing 'recv_can_id' for gripper");
        return false;
      }
      gripper_cfg.send_can_id = gripper["send_can_id"].as<uint32_t>();
      gripper_cfg.recv_can_id = gripper["recv_can_id"].as<uint32_t>();

      // Parse MIT mode parameters (required for gripper too)
      if (!gripper["mit_mode"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode' section for gripper");
        return false;
      }
      if (!gripper["mit_mode"]["kp"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode.kp' for gripper");
        return false;
      }
      if (!gripper["mit_mode"]["kd"]) {
        RCLCPP_ERROR(logger, "Missing 'mit_mode.kd' for gripper");
        return false;
      }
      gripper_cfg.kp = gripper["mit_mode"]["kp"].as<double>();
      gripper_cfg.kd = gripper["mit_mode"]["kd"].as<double>();

      // Parse gripper-specific positions (required if gripper is defined)
      if (!gripper["closed_position"]) {
        RCLCPP_ERROR(logger, "Missing 'closed_position' for gripper");
        return false;
      }
      if (!gripper["open_position"]) {
        RCLCPP_ERROR(logger, "Missing 'open_position' for gripper");
        return false;
      }
      if (!gripper["motor_closed_radians"]) {
        RCLCPP_ERROR(logger, "Missing 'motor_closed_radians' for gripper");
        return false;
      }
      if (!gripper["motor_open_radians"]) {
        RCLCPP_ERROR(logger, "Missing 'motor_open_radians' for gripper");
        return false;
      }
      gripper_cfg.closed_position = gripper["closed_position"].as<double>();
      gripper_cfg.open_position = gripper["open_position"].as<double>();
      gripper_cfg.motor_closed_radians =
          gripper["motor_closed_radians"].as<double>();
      gripper_cfg.motor_open_radians =
          gripper["motor_open_radians"].as<double>();

      gripper_config_ = gripper_cfg;
      RCLCPP_INFO(logger,
                  "Configured gripper: type=%s, send_id=0x%X, recv_id=0x%X, "
                  "kp=%.2f, kd=%.2f",
                  motor_type_str.c_str(), gripper_cfg.send_can_id,
                  gripper_cfg.recv_can_id, gripper_cfg.kp, gripper_cfg.kd);
    }

    // Update the number of joints based on loaded configuration
    num_joints_ = motor_configs_.size();
    if (gripper_config_.has_value()) {
      num_joints_++;  // Add one for the gripper
    }

    RCLCPP_INFO(logger, "Successfully loaded configuration for %zu motors",
                motor_configs_.size());
    return true;

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(logger, "Failed to parse YAML file: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error loading motor configuration: %s", e.what());
    return false;
  }
}

bool OpenArm_v10RTHardware::generate_joint_names() {
  // Generate joint names if not provided
  if (joint_names_.empty() && num_joints_ > 0) {
    for (size_t i = 0; i < num_joints_; ++i) {
      joint_names_.push_back("joint_" + std::to_string(i));
    }
    return true;
  }
  return !joint_names_.empty();
}

void OpenArm_v10RTHardware::can_worker_loop() {
  // This runs in a separate non-RT thread
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "CAN worker thread started");

  // Set thread name for debugging
  pthread_setname_np(pthread_self(), "openarm_can");

  // Alternating pattern: send on even cycles, receive on odd cycles
  // This gives cycle_time / 2 (tx + rx direction)
  const auto cycle_time = std::chrono::microseconds(1600);  // 600Hz base rate

  // Cycle counter for alternating pattern
  uint32_t cycle_counter = 0;

  while (worker_running_) {
    auto cycle_start = std::chrono::steady_clock::now();

    // Alternate between sending and receiving to reduce CAN bus load
    bool send_cycle = (cycle_counter % 2 == 0);
    cycle_counter++;

    // Check for pending mode switch
    if (mode_switch_requested_.exchange(false)) {
      perform_mode_switch_async();
    }

    if (send_cycle) {
      // SEND CYCLE: Read commands from RT thread and send to motors
      auto cmd = command_buffer_.readFromNonRT();
      if (cmd && cmd->valid && cmd->mode != ControlMode::UNINITIALIZED) {
        // Send commands to motors based on current mode
        if (cmd->mode == ControlMode::MIT) {
          // Pack MIT commands
          for (size_t i = 0; i < num_joints_; ++i) {
            mit_params_[i].q = cmd->positions[i];
            mit_params_[i].dq = cmd->velocities[i];
            mit_params_[i].tau = cmd->torques[i];
            mit_params_[i].kp = config_.mit_kp;
            mit_params_[i].kd = config_.mit_kd;
          }
          // Send MIT commands (batch) using RT-safe method
          openarm_rt_->send_mit_batch_rt(mit_params_.data(), num_joints_,
                                         config_.can_timeout_us);

        } else if (cmd->mode == ControlMode::POSITION_VELOCITY) {
          // Pack position/velocity commands
          for (size_t i = 0; i < num_joints_; ++i) {
            posvel_params_[i].position = cmd->positions[i];
            posvel_params_[i].velocity = cmd->velocities[i];
          }
          // Send position/velocity commands (batch) using RT-safe method
          openarm_rt_->send_posvel_batch_rt(posvel_params_.data(), num_joints_,
                                            config_.can_timeout_us);
        }
      } else {
        // No active controller - send refresh command to get motor states
        // This uses the special 0x7FF CAN ID to request state feedback
        openarm_rt_->refresh_all_motors_rt(config_.can_timeout_us);
      }
    } else {
      // RECEIVE CYCLE: Read states from motors
      StateData new_state;
      new_state.valid = false;

      // Receive motor states (batch) using RT-safe method
      std::array<openarm::damiao_motor::StateResult, MAX_JOINTS> motor_states;
      size_t received = openarm_rt_->receive_states_batch_rt(
          motor_states.data(), num_joints_, config_.can_timeout_us);

      if (received > 0) {
        for (size_t i = 0; i < num_joints_; ++i) {
          if (motor_states[i].valid) {
            new_state.positions[i] = motor_states[i].position;
            new_state.velocities[i] = motor_states[i].velocity;
            new_state.torques[i] = motor_states[i].torque;
            new_state.error_codes[i] = motor_states[i].error_code;
          }
        }
        new_state.valid = true;

        // Write to RT buffer
        state_buffer_.writeFromNonRT(new_state);
      }
    }

    // Sleep to maintain cycle time
    auto cycle_end = std::chrono::steady_clock::now();
    auto cycle_duration = cycle_end - cycle_start;

    if (cycle_duration < cycle_time) {
      std::this_thread::sleep_until(cycle_start + cycle_time);
    } else {
      // Log if we're missing cycles (non-RT thread, so logging is OK)
      static int missed_cycles = 0;
      missed_cycles++;

      // Log every 1000 missed cycles with more detail
      if (missed_cycles % 1000 == 0) {
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_duration)
                .count();
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                    "CAN worker thread missed %d cycles. Last cycle took %ld "
                    "us (target: 2500 us). %s cycle.",
                    missed_cycles, duration_us,
                    send_cycle ? "Send" : "Receive");
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "CAN worker thread stopped");
}

bool OpenArm_v10RTHardware::perform_mode_switch_async() {
  // This is called from the async handler (non-RT context)
  ControlMode new_mode = pending_mode_.load();
  ControlMode old_mode = current_mode_.load();

  if (new_mode == old_mode) {
    return true;  // No change needed
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Switching control mode from %d to %d",
              static_cast<int>(old_mode), static_cast<int>(new_mode));

  bool success = false;

  // Perform mode-specific initialization
  if (new_mode == ControlMode::MIT) {
    success = switch_to_mit_mode();
  } else if (new_mode == ControlMode::POSITION_VELOCITY) {
    success = switch_to_position_mode();
  }

  if (success) {
    current_mode_ = new_mode;
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Control mode switch completed successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to switch control mode");
  }

  return success;
}

bool OpenArm_v10RTHardware::switch_to_mit_mode() {
  // If motors are not enabled yet (first mode switch), enable them first
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Enabling motors for MIT mode");

    // Enable motors - this is RT-safe and non-blocking
    size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
    if (enabled != openarm_rt_->get_motor_count()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to enable all motors: %zu/%zu", enabled,
                   openarm_rt_->get_motor_count());
      return false;
    }

    // Clear command buffer - mark as invalid so we don't send garbage
    CommandData invalid_cmd;
    invalid_cmd.valid = false;
    command_buffer_.writeFromNonRT(invalid_cmd);
  }

  // Initialize command interfaces with current motor states to avoid jumps
  // This ensures the first MIT command sent will match current state
  for (size_t i = 0; i < num_joints_; ++i) {
    pos_commands_[i] = pos_states_[i];
    vel_commands_[i] = vel_states_[i];
    tau_commands_[i] = tau_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Seeded command interfaces with current motor states for smooth "
              "transition");

  // Write CTRL_MODE parameter to switch to MIT mode (value = 1)
  size_t written = openarm_rt_->write_param_all_rt(
      openarm::damiao_motor::RID::CTRL_MODE, 1, 1000);

  if (written != openarm_rt_->get_motor_count()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to switch all motors to MIT mode: %zu/%zu", written,
                 openarm_rt_->get_motor_count());
    return false;
  }

  return true;
}

bool OpenArm_v10RTHardware::switch_to_position_mode() {
  // If motors are not enabled yet (first mode switch), enable them first
  if (current_mode_ == ControlMode::UNINITIALIZED) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "Enabling motors for Position/Velocity mode");

    // Enable motors - this is RT-safe and non-blocking
    size_t enabled = openarm_rt_->enable_all_motors_rt(1000);
    if (enabled != openarm_rt_->get_motor_count()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                   "Failed to enable all motors: %zu/%zu", enabled,
                   openarm_rt_->get_motor_count());
      return false;
    }

    // Clear command buffer - mark as invalid so we don't send garbage
    CommandData invalid_cmd;
    invalid_cmd.valid = false;
    command_buffer_.writeFromNonRT(invalid_cmd);
  }

  // Initialize command interfaces with current motor states to avoid jumps
  // This ensures the first Position/Velocity command sent will match current
  // state
  for (size_t i = 0; i < num_joints_; ++i) {
    pos_commands_[i] = pos_states_[i];
    vel_commands_[i] = vel_states_[i];
    tau_commands_[i] = tau_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Seeded command interfaces with current motor states for smooth "
              "transition");

  // Write CTRL_MODE parameter to switch to Position/Velocity mode (value = 2)
  size_t written = openarm_rt_->write_param_all_rt(
      openarm::damiao_motor::RID::CTRL_MODE, 2, 1000);

  if (written != openarm_rt_->get_motor_count()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("OpenArm_v10RTHardware"),
        "Failed to switch all motors to Position/Velocity mode: %zu/%zu",
        written, openarm_rt_->get_motor_count());
    return false;
  }

  return true;
}

ControlMode OpenArm_v10RTHardware::determine_mode_from_interfaces(
    const std::vector<std::string>& interfaces) {
  bool has_position = false;
  bool has_velocity = false;
  bool has_effort = false;

  for (const auto& interface : interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
      has_position = true;
    } else if (interface.find(hardware_interface::HW_IF_VELOCITY) !=
               std::string::npos) {
      has_velocity = true;
    } else if (interface.find(hardware_interface::HW_IF_EFFORT) !=
               std::string::npos) {
      has_effort = true;
    }
  }

  // Determine mode based on interface combination
  // Effort (torque) interface claimed -> MIT mode for impedance control
  if (has_effort) {
    return ControlMode::MIT;
  }

  // Position interface claimed without effort -> Position-Velocity mode
  if (has_position && !has_effort) {
    return ControlMode::POSITION_VELOCITY;
  }

  // Invalid or unclear combination
  return ControlMode::UNINITIALIZED;
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10RTHardware,
                       hardware_interface::SystemInterface)
