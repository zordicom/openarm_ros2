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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

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

  if (!openarm_rt_->init(config_.can_interface, false)) {  // false = no CAN-FD
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to initialize RT-safe OpenArm interface");
    return CallbackReturn::ERROR;
  }

  // TODO: Add motors based on configuration
  // This would typically come from the motor_config_file_
  // For now, we'll need to implement load_motor_config_from_yaml to populate
  // motors

  // Set initial control mode
  current_mode_ = ControlMode::POSITION_VELOCITY;

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

  // Enable motors (using RT-safe method even in non-RT context for consistency)
  if (openarm_rt_->enable_all_motors_rt(1000) !=
      openarm_rt_->get_motor_count()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to enable all motors");
    return CallbackReturn::ERROR;
  }

  // Initialize timing
  last_read_time_ = std::chrono::steady_clock::now();
  last_write_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Hardware interface activated successfully");

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
  // This is called from RT context - just copy from the realtime buffer
  auto state = state_buffer_.readFromRT();

  if (state && state->valid) {
    for (size_t i = 0; i < num_joints_; ++i) {
      pos_states_[i] = state->positions[i];
      vel_states_[i] = state->velocities[i];
      tau_states_[i] = state->torques[i];

      // Check for motor errors (RT-safe)
      if (state->error_codes[i] != 0) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("OpenArm_v10RTHardware"),
                              steady_clock, LOG_THROTTLE_MS,
                              "Motor %zu error code: %u", i,
                              state->error_codes[i]);
      }
    }
  }

  // Monitor timing (RT-safe)
  auto now = std::chrono::steady_clock::now();
  auto read_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                           now - last_read_time_)
                           .count();

  if (read_duration > config_.max_cycle_time_us) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10RTHardware"),
                         steady_clock, LOG_THROTTLE_MS,
                         "Read cycle time exceeded: %ld us > %d us",
                         read_duration, config_.max_cycle_time_us);
  }

  last_read_time_ = now;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10RTHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
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

  // Monitor timing (RT-safe)
  auto now = std::chrono::steady_clock::now();
  auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            now - last_write_time_)
                            .count();

  if (write_duration > config_.max_cycle_time_us) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("OpenArm_v10RTHardware"),
                         steady_clock, LOG_THROTTLE_MS,
                         "Write cycle time exceeded: %ld us > %d us",
                         write_duration, config_.max_cycle_time_us);
  }

  last_write_time_ = now;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenArm_v10RTHardware::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  // Determine new mode from interfaces
  ControlMode new_mode = determine_mode_from_interfaces(start_interfaces);

  if (new_mode == ControlMode::UNINITIALIZED) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Invalid interface combination for mode switch");
    return hardware_interface::return_type::ERROR;
  }

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
  // TODO: Implement YAML loading for motor configuration
  // This would typically load motor-specific parameters like:
  // - Motor IDs
  // - Gear ratios
  // - Limits (position, velocity, torque)
  // - PID gains for position mode
  // - MIT mode parameters

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Loading motor configuration from %s", file_path.c_str());

  return true;  // Placeholder
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

  const auto cycle_time = std::chrono::microseconds(1000);  // 1kHz

  while (worker_running_) {
    auto cycle_start = std::chrono::steady_clock::now();

    // Check for pending mode switch
    if (mode_switch_requested_.exchange(false)) {
      perform_mode_switch_async();
    }

    // Read commands from RT thread
    auto cmd = command_buffer_.readFromNonRT();
    if (cmd && cmd->valid) {
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
    }

    // Read states from motors
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

    // Sleep to maintain cycle time
    auto cycle_end = std::chrono::steady_clock::now();
    auto cycle_duration = cycle_end - cycle_start;

    if (cycle_duration < cycle_time) {
      std::this_thread::sleep_until(cycle_start + cycle_time);
    } else {
      // Log if we're missing cycles (non-RT thread, so logging is OK)
      static int missed_cycles = 0;
      if (++missed_cycles % 1000 == 0) {  // Log every 1000 missed cycles
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                    "CAN worker thread missed %d cycles", missed_cycles);
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
  // Send mode switch commands to motors using RT-safe method
  return openarm_rt_->set_mode_all_rt(
      openarm::can::RTSafeOpenArm::ControlMode::MIT, 1000);
}

bool OpenArm_v10RTHardware::switch_to_position_mode() {
  // Send mode switch commands to motors using RT-safe method
  return openarm_rt_->set_mode_all_rt(
      openarm::can::RTSafeOpenArm::ControlMode::POSITION_VELOCITY, 1000);
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
  if (has_position && has_velocity && has_effort) {
    // All interfaces claimed - MIT mode
    return ControlMode::MIT;
  } else if (has_position && has_velocity && !has_effort) {
    // Position and velocity only - Position/Velocity mode
    return ControlMode::POSITION_VELOCITY;
  } else {
    // Invalid combination
    return ControlMode::UNINITIALIZED;
  }
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10RTHardware,
                       hardware_interface::SystemInterface)
