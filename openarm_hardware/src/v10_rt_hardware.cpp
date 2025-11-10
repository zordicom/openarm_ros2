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

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>

#include "openarm/realtime/can_transport.hpp"
#include "openarm/realtime/canfd_transport.hpp"
#include <sstream>
#include <thread>

#include "rclcpp/logging.hpp"
#include "realtime_tools/realtime_helpers.hpp"

// RT-safe logging - just skip the throttling in RT context for simplicity
static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

namespace openarm_hardware {

OpenArm_v10RTHardware::OpenArm_v10RTHardware() {
  // Pre-allocate command and state buffers
  command_buffer_.writeFromNonRT(CommandData{});
  state_buffer_.writeFromNonRT(StateData{});

  // Initialize stats logging timestamp
  last_stats_log_ = std::chrono::steady_clock::now();
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
  // Create appropriate transport (CAN or CAN-FD)
  try {
    std::unique_ptr<openarm::realtime::IOpenArmTransport> transport;
    if (controller_config_.can_fd) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Initializing with CAN-FD transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::CANFDTransport>(config_.can_interface);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
                  "Initializing with standard CAN transport on %s",
                  config_.can_interface.c_str());
      transport = std::make_unique<openarm::realtime::CANTransport>(config_.can_interface);
    }

    // Create RT-safe OpenArm interface with transport
    openarm_rt_ = std::make_unique<openarm::realtime::OpenArm>(std::move(transport));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHardware"),
                 "Failed to initialize OpenArm: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Add motors to RT-safe wrapper based on configuration
  for (const auto& motor : controller_config_.arm_joints) {
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
  if (controller_config_.gripper_joint.has_value()) {
    int gripper_idx =
        openarm_rt_->add_motor(controller_config_.gripper_joint->motor_type,
                               controller_config_.gripper_joint->send_can_id,
                               controller_config_.gripper_joint->recv_can_id);
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

  // Configure worker thread as lower-priority RT thread
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
      for (size_t i = 0; i < config_.cpu_affinity.size(); i++) {
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
OpenArm_v10RTHardware::export_command_interfaces() {
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

hardware_interface::return_type OpenArm_v10RTHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Start timing
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);

  // This is called from RT context - just copy from the realtime buffer
  auto state = state_buffer_.readFromRT();

  if (state && state->valid) {
    for (size_t i = 0; i < num_joints_; i++) {
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

  // End timing and update stats
  clock_gettime(CLOCK_MONOTONIC, &end);

  // Handle nanosecond wraparound correctly
  int64_t sec_diff = end.tv_sec - start.tv_sec;
  int64_t nsec_diff = end.tv_nsec - start.tv_nsec;
  if (nsec_diff < 0) {
    sec_diff--;
    nsec_diff += 1000000000L;
  }
  uint64_t duration_ns = sec_diff * 1000000000ULL + nsec_diff;

  rt_stats_.read_count.fetch_add(1, std::memory_order_relaxed);
  rt_stats_.total_read_ns.fetch_add(duration_ns, std::memory_order_relaxed);

  // Update max (lock-free)
  uint64_t current_max = rt_stats_.max_read_ns.load(std::memory_order_relaxed);
  while (duration_ns > current_max &&
         !rt_stats_.max_read_ns.compare_exchange_weak(current_max, duration_ns,
                                                       std::memory_order_relaxed)) {
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10RTHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Start timing
  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);

  // This is called from RT context - just write to the realtime buffer
  CommandData cmd;
  cmd.mode = current_mode_.load();

  // If a mode switch is pending, don't send commands until it completes
  // This prevents sending stale commands in the wrong mode
  if (mode_switch_requested_.load()) {
    cmd.valid = false;
  } else {
    cmd.valid = true;
    for (size_t i = 0; i < num_joints_; i++) {
      cmd.positions[i] = pos_commands_[i];
      cmd.velocities[i] = vel_commands_[i];
      cmd.torques[i] = tau_commands_[i];
    }
  }

  command_buffer_.writeFromNonRT(cmd);

  // End timing and update stats
  clock_gettime(CLOCK_MONOTONIC, &end);

  // Handle nanosecond wraparound correctly
  int64_t sec_diff = end.tv_sec - start.tv_sec;
  int64_t nsec_diff = end.tv_nsec - start.tv_nsec;
  if (nsec_diff < 0) {
    sec_diff--;
    nsec_diff += 1000000000L;
  }
  uint64_t duration_ns = sec_diff * 1000000000ULL + nsec_diff;

  rt_stats_.write_count.fetch_add(1, std::memory_order_relaxed);
  rt_stats_.total_write_ns.fetch_add(duration_ns, std::memory_order_relaxed);

  // Update max (lock-free)
  uint64_t current_max = rt_stats_.max_write_ns.load(std::memory_order_relaxed);
  while (duration_ns > current_max &&
         !rt_stats_.max_write_ns.compare_exchange_weak(current_max, duration_ns,
                                                        std::memory_order_relaxed)) {
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
  auto logger = rclcpp::get_logger("OpenArm_v10RTHardware");

  try {
    // Parse CAN interface settings from hardware parameters
    auto it = info.hardware_parameters.find("can_interface");
    if (it == info.hardware_parameters.end() || it->second.empty()) {
      RCLCPP_ERROR(logger, "Required parameter 'can_interface' not provided");
      return false;
    }
    config_.can_interface = it->second;
    controller_config_.can_iface = it->second;

    it = info.hardware_parameters.find("can_fd");
    if (it != info.hardware_parameters.end()) {
      controller_config_.can_fd = parse_bool_param(it->second);
    } else {
      controller_config_.can_fd = false;
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

        auto motor_type_it = params.find("motor_type");
        if (motor_type_it == params.end()) {
          RCLCPP_ERROR(logger,
                       "Gripper joint '%s' missing 'motor_type' parameter",
                       joint.name.c_str());
          return false;
        }
        gripper.motor_type = parse_motor_type_param(motor_type_it->second);

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

        controller_config_.gripper_joint = gripper;

        RCLCPP_INFO(logger, "Configured gripper joint: %s", joint.name.c_str());
      } else {
        // Parse arm joint configuration
        MotorConfig motor;
        motor.name = joint.name;

        auto motor_type_it = params.find("motor_type");
        if (motor_type_it == params.end()) {
          RCLCPP_ERROR(logger, "Arm joint '%s' missing 'motor_type' parameter",
                       joint.name.c_str());
          return false;
        }
        motor.type = parse_motor_type_param(motor_type_it->second);

        motor.send_can_id = std::stoul(params.at("send_can_id"), nullptr, 0);
        motor.recv_can_id = std::stoul(params.at("recv_can_id"), nullptr, 0);
        motor.kp = std::stod(params.at("kp"));
        motor.kd = std::stod(params.at("kd"));
        motor.max_velocity = std::stod(params.at("max_velocity"));

        controller_config_.arm_joints.push_back(motor);

        RCLCPP_INFO(logger, "Configured arm joint: %s", joint.name.c_str());
      }
    }

    if (controller_config_.arm_joints.empty()) {
      RCLCPP_ERROR(logger, "No arm joints configured");
      return false;
    }

    // Build joint names vector from config
    joint_names_.clear();
    for (const auto& motor : controller_config_.arm_joints) {
      joint_names_.push_back(motor.name);
    }
    if (controller_config_.gripper_joint.has_value()) {
      joint_names_.push_back(controller_config_.gripper_joint->name);
    }
    num_joints_ = joint_names_.size();

    RCLCPP_INFO(logger, "Configured %zu arm joints and %s gripper",
                controller_config_.arm_joints.size(),
                controller_config_.gripper_joint.has_value() ? "1" : "0");

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Failed to parse configuration: %s", e.what());
    return false;
  }
}

void OpenArm_v10RTHardware::can_worker_loop() {
  // This runs in a separate lower-priority RT thread
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "CAN worker thread started");

  // Set thread name for debugging
  pthread_setname_np(pthread_self(), "can_rw_worker");

  // Alternating pattern: send on even cycles, receive on odd cycles
  // This gives motors time to process commands before reading responses
  // Cycle time: 1600us = 625Hz base rate (but 312.5Hz effective per direction)
  const int64_t cycle_time_ns = 1600000;  // 1600us = 1.6ms = 625Hz

  // Cycle counter for alternating pattern
  uint32_t cycle_counter = 0;

  while (worker_running_) {
    // Mark cycle start
    struct timespec cycle_start, cycle_end;
    clock_gettime(CLOCK_MONOTONIC, &cycle_start);

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
        size_t sent = 0;
        if (cmd->mode == ControlMode::MIT) {
          // Pack MIT commands
          for (size_t i = 0; i < num_joints_; i++) {
            mit_params_[i].q = cmd->positions[i];
            mit_params_[i].dq = cmd->velocities[i];
            mit_params_[i].tau = cmd->torques[i];
            mit_params_[i].kp = controller_config_.arm_joints[i].kp;
            mit_params_[i].kd = controller_config_.arm_joints[i].kd;
          }
          // Send MIT commands (batch) using RT-safe method
          sent = openarm_rt_->send_mit_batch_rt(mit_params_.data(), num_joints_,
                                                config_.can_timeout_us);

        } else if (cmd->mode == ControlMode::POSITION_VELOCITY) {
          // Pack position/velocity commands
          for (size_t i = 0; i < num_joints_; i++) {
            posvel_params_[i].q = cmd->positions[i];
            posvel_params_[i].dq = cmd->velocities[i];
          }
          // Send position/velocity commands (batch) using RT-safe method
          sent = openarm_rt_->send_posvel_batch_rt(posvel_params_.data(), num_joints_,
                                                   config_.can_timeout_us);
        }

        // Track dropped frames (TX buffer full)
        if (sent < num_joints_) {
          rt_stats_.tx_dropped.fetch_add(num_joints_ - sent,
                                         std::memory_order_relaxed);
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

      // Track missing feedback frames (RX buffer empty or frames lost)
      if (received < num_joints_) {
        rt_stats_.rx_dropped.fetch_add(num_joints_ - received,
                                       std::memory_order_relaxed);
      }

      if (received > 0) {
        for (size_t i = 0; i < num_joints_; i++) {
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

    // Mark cycle end and compute duration
    clock_gettime(CLOCK_MONOTONIC, &cycle_end);

    // Handle nanosecond wraparound correctly
    int64_t sec_diff = cycle_end.tv_sec - cycle_start.tv_sec;
    int64_t nsec_diff = cycle_end.tv_nsec - cycle_start.tv_nsec;
    if (nsec_diff < 0) {
      sec_diff--;
      nsec_diff += 1000000000L;
    }
    uint64_t cycle_duration_ns = sec_diff * 1000000000ULL + nsec_diff;

    rt_stats_.worker_cycles.fetch_add(1, std::memory_order_relaxed);
    rt_stats_.total_worker_cycle_ns.fetch_add(cycle_duration_ns,
                                              std::memory_order_relaxed);

    // Update max (lock-free)
    uint64_t current_max =
        rt_stats_.max_worker_cycle_ns.load(std::memory_order_relaxed);
    while (cycle_duration_ns > current_max &&
           !rt_stats_.max_worker_cycle_ns.compare_exchange_weak(
               current_max, cycle_duration_ns, std::memory_order_relaxed)) {
    }

    // Check for deadline miss (cycle took longer than allocated time)
    if (cycle_duration_ns > static_cast<uint64_t>(cycle_time_ns)) {
      rt_stats_.worker_deadline_misses.fetch_add(1, std::memory_order_relaxed);
    }

    // Periodically log RT stats (non-RT safe, but low priority thread)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                       now - last_stats_log_)
                       .count();
    if (elapsed >= STATS_LOG_INTERVAL_SEC) {
      log_rt_stats();
      last_stats_log_ = now;
    }

    // Sleep to maintain cycle time (use relative sleep like working version)
    if (cycle_duration_ns < static_cast<uint64_t>(cycle_time_ns)) {
      int64_t sleep_ns = cycle_time_ns - cycle_duration_ns;
      struct timespec sleep_time;
      sleep_time.tv_sec = sleep_ns / 1000000000L;
      sleep_time.tv_nsec = sleep_ns % 1000000000L;
      clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, nullptr);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "CAN worker thread stopped");
}

bool OpenArm_v10RTHardware::perform_mode_switch_async() {
  // This is called from the lower-priority RT worker thread
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
  for (size_t i = 0; i < num_joints_; i++) {
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
  for (size_t i = 0; i < num_joints_; i++) {
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
  bool has_effort = false;

  for (const auto& interface : interfaces) {
    if (interface.find(hardware_interface::HW_IF_POSITION) !=
        std::string::npos) {
      has_position = true;
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

void OpenArm_v10RTHardware::log_rt_stats() {
  // Load all stats atomically
  uint64_t read_count = rt_stats_.read_count.load(std::memory_order_relaxed);
  uint64_t write_count = rt_stats_.write_count.load(std::memory_order_relaxed);
  uint64_t worker_cycles =
      rt_stats_.worker_cycles.load(std::memory_order_relaxed);

  uint64_t max_read_ns = rt_stats_.max_read_ns.load(std::memory_order_relaxed);
  uint64_t max_write_ns =
      rt_stats_.max_write_ns.load(std::memory_order_relaxed);
  uint64_t max_worker_ns =
      rt_stats_.max_worker_cycle_ns.load(std::memory_order_relaxed);

  uint64_t total_read_ns =
      rt_stats_.total_read_ns.load(std::memory_order_relaxed);
  uint64_t total_write_ns =
      rt_stats_.total_write_ns.load(std::memory_order_relaxed);
  uint64_t total_worker_ns =
      rt_stats_.total_worker_cycle_ns.load(std::memory_order_relaxed);

  uint64_t deadline_misses =
      rt_stats_.worker_deadline_misses.load(std::memory_order_relaxed);
  uint64_t tx_dropped =
      rt_stats_.tx_dropped.load(std::memory_order_relaxed);
  uint64_t rx_dropped =
      rt_stats_.rx_dropped.load(std::memory_order_relaxed);

  // Calculate averages
  double avg_read_us =
      read_count > 0 ? (total_read_ns / static_cast<double>(read_count)) / 1000.0
                     : 0.0;
  double avg_write_us = write_count > 0
                            ? (total_write_ns / static_cast<double>(write_count)) /
                                  1000.0
                            : 0.0;
  double avg_worker_us =
      worker_cycles > 0
          ? (total_worker_ns / static_cast<double>(worker_cycles)) / 1000.0
          : 0.0;

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "=== RT Performance Stats ===");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "Main RT thread (read/write):");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "  read():  count=%lu, avg=%.2f us, max=%.2f us", read_count,
              avg_read_us, max_read_ns / 1000.0);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "  write(): count=%lu, avg=%.2f us, max=%.2f us", write_count,
              avg_write_us, max_write_ns / 1000.0);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "CAN worker thread:");
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "  cycles=%lu, avg=%.2f us, max=%.2f us, deadline_misses=%lu",
              worker_cycles, avg_worker_us, max_worker_ns / 1000.0,
              deadline_misses);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "  tx_dropped=%lu (TX buffer full)", tx_dropped);
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHardware"),
              "  rx_dropped=%lu (missing feedback)", rx_dropped);

  if (deadline_misses > 0) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "  WARNING: %lu deadline misses detected (%.2f%%)", deadline_misses,
                100.0 * deadline_misses / static_cast<double>(worker_cycles));
  }
  if (tx_dropped > 0) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "  WARNING: %lu TX frames dropped (CAN TX buffer full)", tx_dropped);
  }
  if (rx_dropped > 0) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHardware"),
                "  WARNING: %lu RX frames missing (no feedback received)", rx_dropped);
  }
}

}  // namespace openarm_hardware

// Register the hardware interface
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10RTHardware,
                       hardware_interface::SystemInterface)
