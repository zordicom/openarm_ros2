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
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArm_v10RTHW::OpenArm_v10RTHW() = default;

OpenArm_v10RTHW::~OpenArm_v10RTHW() {
  // Ensure thread is stopped
  if (thread_running_) {
    thread_running_ = false;
    if (can_thread_ && can_thread_->joinable()) {
      can_thread_->join();
    }
  }
}

bool OpenArm_v10RTHW::parse_config(
    const hardware_interface::HardwareInfo& info) {
  try {
    // Parse CAN interface settings
    auto it = info.hardware_parameters.find("can_interface");
    if (it == info.hardware_parameters.end() || it->second.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                   "Required parameter 'can_interface' not provided");
      return false;
    }
    config_.can_iface = it->second;

    // Parse CAN-FD setting
    it = info.hardware_parameters.find("can_fd");
    config_.can_fd = (it != info.hardware_parameters.end())
                         ? parse_bool_param(it->second)
                         : false;

    // Parse thread cycle time (microseconds)
    it = info.hardware_parameters.find("can_thread_cycle_us");
    if (it != info.hardware_parameters.end()) {
      thread_cycle_time_ = std::chrono::microseconds(std::stoi(it->second));
    }

    // Parse HardwareConfig RT settings
    hw_config_.can_interface = config_.can_iface;

    it = info.hardware_parameters.find("can_timeout_us");
    if (it != info.hardware_parameters.end()) {
      hw_config_.can_timeout_us = std::stoi(it->second);
    }

    // Parse thread priority
    it = info.hardware_parameters.find("rt_priority");
    if (it != info.hardware_parameters.end()) {
      hw_config_.rt_priority = std::stoi(it->second);
      thread_priority_ = hw_config_.rt_priority;
    }

    // Parse CPU affinity (comma-separated list)
    it = info.hardware_parameters.find("cpu_affinity");
    if (it != info.hardware_parameters.end() && !it->second.empty()) {
      std::stringstream ss(it->second);
      std::string cpu_str;
      while (std::getline(ss, cpu_str, ',')) {
        hw_config_.cpu_affinity.push_back(std::stoi(cpu_str));
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("OpenArm_v10RTHW"),
        "CAN interface: %s, CAN-FD: %s, Thread cycle: %ld us, Priority: %d",
        config_.can_iface.c_str(), config_.can_fd ? "enabled" : "disabled",
        thread_cycle_time_.count(), thread_priority_);

    // Parse joint-level parameters (same as v10_simple_hardware)
    for (const auto& joint : info.joints) {
      const auto& params = joint.parameters;

      auto is_gripper_it = params.find("is_gripper");
      bool is_gripper = (is_gripper_it != params.end()) &&
                        parse_bool_param(is_gripper_it->second);

      if (is_gripper) {
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
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                    "Configured gripper joint: %s", joint.name.c_str());
      } else {
        MotorConfig motor;
        motor.name = joint.name;

        motor.send_can_id = std::stoul(params.at("send_can_id"), nullptr, 0);
        motor.recv_can_id = std::stoul(params.at("recv_can_id"), nullptr, 0);
        motor.kp = std::stod(params.at("kp"));
        motor.kd = std::stod(params.at("kd"));
        motor.max_velocity = std::stod(params.at("max_velocity"));

        config_.arm_joints.push_back(motor);
        RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                    "Configured arm joint: %s", joint.name.c_str());
      }
    }

    if (config_.arm_joints.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                   "No arm joints configured");
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                "Configured %zu total joints (%zu arm + %zu gripper)",
                num_joints_, config_.arm_joints.size(),
                config_.gripper_joint.has_value() ? size_t(1) : size_t(0));

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                 "Failed to parse configuration: %s", e.what());
    return false;
  }
}

hardware_interface::CallbackReturn OpenArm_v10RTHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  num_joints_ =
      config_.arm_joints.size() + (config_.gripper_joint.has_value() ? 1 : 0);

  if (num_joints_ >= MAX_JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                 "Too many joints. Got %ld, max allowed %ld", num_joints_,
                 MAX_JOINTS);
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < config_.arm_joints.size(); i++) {
    joint_names_[i] = config_.arm_joints[i].name;
  }

  if (config_.gripper_joint.has_value()) {
    joint_names_[num_joints_ - 1] = config_.gripper_joint.value().name;
  }

  // Initialize RT OpenArm with transport
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
              "Initializing RT OpenArm on %s...", config_.can_iface.c_str());

  try {
    std::unique_ptr<openarm::realtime::IOpenArmTransport> transport;

    if (config_.can_fd) {
      transport = std::make_unique<openarm::realtime::can::CANFDSocket>(
          config_.can_iface);
    } else {
      transport = std::make_unique<openarm::realtime::can::CANSocket>(
          config_.can_iface);
    }

    openarm_rt_ =
        std::make_unique<openarm::realtime::OpenArm>(std::move(transport));

    // Add motors to RT interface
    for (const auto& motor : config_.arm_joints) {
      int motor_idx =
          openarm_rt_->add_motor(motor.send_can_id, motor.recv_can_id);
      if (motor_idx < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                     "Failed to add motor with CAN IDs 0x%03X/0x%03X",
                     motor.send_can_id, motor.recv_can_id);
        return CallbackReturn::ERROR;
      }
    }

    if (config_.gripper_joint.has_value()) {
      const auto& gripper = config_.gripper_joint.value();
      int motor_idx =
          openarm_rt_->add_motor(gripper.send_can_id, gripper.recv_can_id);
      if (motor_idx < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                     "Failed to add gripper motor with CAN IDs 0x%03X/0x%03X",
                     gripper.send_can_id, gripper.recv_can_id);
        return CallbackReturn::ERROR;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                 "Failed to initialize RT OpenArm: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
              "RT OpenArm initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10RTHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"), "Configuring motors...");

  // Set all motors to MIT mode
  openarm_rt_->set_mode_all_rt(openarm::realtime::ControlMode::MIT, 1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Clear any pending messages
  openarm::damiao_motor::StateResult states[MAX_JOINTS];
  openarm_rt_->receive_states_batch_rt(states, MAX_JOINTS, 1000);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArm_v10RTHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
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
OpenArm_v10RTHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], "kp", &kp_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], "kd", &kd_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArm_v10RTHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"), "Activating motors...");

  // Enable all motors
  ssize_t enabled = openarm_rt_->enable_all_motors_rt(1000);
  if (enabled != static_cast<ssize_t>(num_joints_)) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                 "Failed to enable all motors (enabled %zd/%zu)", enabled,
                 num_joints_);
    return CallbackReturn::ERROR;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Read initial positions
  openarm_rt_->refresh_all_motors_rt(1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  openarm::damiao_motor::StateResult states[MAX_JOINTS];
  ssize_t received =
      openarm_rt_->receive_states_batch_rt(states, MAX_JOINTS, 1000);

  for (size_t i = 0; i < num_joints_ && i < static_cast<size_t>(received);
       ++i) {
    if (states[i].valid) {
      pos_commands_[i] = states[i].position;
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                  "Joint %zu initialized to position: %.3f rad", i,
                  pos_commands_[i]);
    }
  }

  // Initialize command buffers and command vectors with configured defaults
  int write_idx = cmd_write_idx_.load(std::memory_order_acquire);
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    cmd_buffers_[write_idx][i].position = pos_commands_[i];
    cmd_buffers_[write_idx][i].velocity = 0.0;
    cmd_buffers_[write_idx][i].torque = 0.0;
    cmd_buffers_[write_idx][i].kp = config_.arm_joints[i].kp;
    cmd_buffers_[write_idx][i].kd = config_.arm_joints[i].kd;

    // Initialize command interface values with defaults
    kp_commands_[i] = config_.arm_joints[i].kp;
    kd_commands_[i] = config_.arm_joints[i].kd;
  }

  if (config_.gripper_joint.has_value()) {
    size_t idx = config_.arm_joints.size();
    cmd_buffers_[write_idx][idx].position = pos_commands_[idx];
    cmd_buffers_[write_idx][idx].velocity = 0.0;
    cmd_buffers_[write_idx][idx].torque = 0.0;
    cmd_buffers_[write_idx][idx].kp = config_.gripper_joint->kp;
    cmd_buffers_[write_idx][idx].kd = config_.gripper_joint->kd;

    // Initialize command interface values with defaults
    kp_commands_[idx] = config_.gripper_joint->kp;
    kd_commands_[idx] = config_.gripper_joint->kd;
  }

  // Lock memory to prevent page faults (RT-safe)
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHW"),
                "Failed to lock memory (mlockall): %s. Consider running with "
                "CAP_IPC_LOCK capability or increasing RLIMIT_MEMLOCK",
                strerror(errno));
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                "Successfully locked process memory (mlockall)");
  }

  // Start background CAN thread
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
              "Starting background CAN thread...");
  thread_running_ = true;
  can_thread_ =
      std::make_unique<std::thread>(&OpenArm_v10RTHW::can_thread_loop, this);

  // Set thread priority
  struct sched_param param;
  param.sched_priority = thread_priority_;
  if (pthread_setschedparam(can_thread_->native_handle(), SCHED_FIFO, &param) !=
      0) {
    RCLCPP_WARN(
        rclcpp::get_logger("OpenArm_v10RTHW"),
        "Failed to set CAN thread priority (may need CAP_SYS_NICE capability)");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                "CAN thread priority set to %d", thread_priority_);
  }

  // Set CPU affinity for background thread
  if (!hw_config_.cpu_affinity.empty()) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    for (int cpu : hw_config_.cpu_affinity) {
      CPU_SET(cpu, &cpuset);
    }

    if (pthread_setaffinity_np(can_thread_->native_handle(), sizeof(cpu_set_t),
                               &cpuset) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW"),
                   "Failed to set CAN thread CPU affinity: %s",
                   strerror(errno));
    } else {
      // Format CPU list for logging
      std::ostringstream cpu_list_str;
      for (size_t i = 0; i < hw_config_.cpu_affinity.size(); i++) {
        if (i > 0) cpu_list_str << ",";
        cpu_list_str << hw_config_.cpu_affinity[i];
      }
      RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                  "CAN thread pinned to CPUs: %s", cpu_list_str.str().c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"), "Motors activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArm_v10RTHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"), "Deactivating motors...");

  // Stop background thread
  if (thread_running_) {
    thread_running_ = false;
    if (can_thread_ && can_thread_->joinable()) {
      can_thread_->join();
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"),
                "Background thread stopped");
  }

  // Disable all motors
  openarm_rt_->disable_all_motors_rt(1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW"), "Motors deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArm_v10RTHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Swap state buffers (lock-free)
  swap_state_buffers();

  // Read from the state buffer that was just written by the background thread
  int read_idx = state_read_idx_.load(std::memory_order_acquire);

  bool has_error = false;
  for (size_t i = 0; i < num_joints_; ++i) {
    const auto& state = state_buffers_[read_idx][i];

    if (!state.valid) {
      // Keep previous values if state is invalid
      continue;
    }

    pos_states_[i] = state.position;
    vel_states_[i] = state.velocity;
    tau_states_[i] = state.torque;

    // Check for unrecoverable errors
    if (state.error_code != 0) {
      RCLCPP_ERROR_THROTTLE(
          rclcpp::get_logger("OpenArm_v10RTHW"), *rclcpp::Clock::make_shared(),
          1000, "Joint %zu has error code: 0x%02X", i, state.error_code);
      has_error = true;
    }
  }

  if (has_error) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArm_v10RTHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Write commands to buffer (lock-free)
  int write_idx = cmd_write_idx_.load(std::memory_order_acquire);

  // Update arm commands (including dynamic kp/kd)
  for (size_t i = 0; i < config_.arm_joints.size(); ++i) {
    cmd_buffers_[write_idx][i].position = pos_commands_[i];
    cmd_buffers_[write_idx][i].velocity = vel_commands_[i];
    cmd_buffers_[write_idx][i].torque = tau_commands_[i];

    // Update kp/kd: use commanded value if non-zero, otherwise use default
    cmd_buffers_[write_idx][i].kp =
        (kp_commands_[i] > 0.0) ? kp_commands_[i] : config_.arm_joints[i].kp;
    cmd_buffers_[write_idx][i].kd =
        (kd_commands_[i] > 0.0) ? kd_commands_[i] : config_.arm_joints[i].kd;
  }

  // Update gripper commands if enabled
  if (config_.gripper_joint.has_value()) {
    size_t idx = config_.arm_joints.size();
    const auto& gripper = config_.gripper_joint.value();

    // Convert joint position to motor radians
    double motor_pos = gripper.to_radians(pos_commands_[idx]);

    cmd_buffers_[write_idx][idx].position = motor_pos;
    cmd_buffers_[write_idx][idx].velocity = vel_commands_[idx];
    cmd_buffers_[write_idx][idx].torque = tau_commands_[idx];

    // Update kp/kd: use commanded value if non-zero, otherwise use default
    cmd_buffers_[write_idx][idx].kp =
        (kp_commands_[idx] > 0.0) ? kp_commands_[idx] : gripper.kp;
    cmd_buffers_[write_idx][idx].kd =
        (kd_commands_[idx] > 0.0) ? kd_commands_[idx] : gripper.kd;
  }

  // Swap command buffers (lock-free)
  swap_command_buffers();

  return hardware_interface::return_type::OK;
}

void OpenArm_v10RTHW::swap_state_buffers() {
  // Atomic swap: controller reads what thread wrote
  int current_read = state_read_idx_.load(std::memory_order_acquire);
  int current_write = state_write_idx_.load(std::memory_order_acquire);

  state_read_idx_.store(current_write, std::memory_order_release);
  state_write_idx_.store(current_read, std::memory_order_release);
}

void OpenArm_v10RTHW::swap_command_buffers() {
  // Atomic swap: thread reads what controller wrote
  int current_read = cmd_read_idx_.load(std::memory_order_acquire);
  int current_write = cmd_write_idx_.load(std::memory_order_acquire);

  cmd_read_idx_.store(current_write, std::memory_order_release);
  cmd_write_idx_.store(current_read, std::memory_order_release);
}

void OpenArm_v10RTHW::can_thread_loop() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW_Thread"),
              "CAN thread started with %ld us cycle time",
              thread_cycle_time_.count());

  openarm::damiao_motor::MITParam mit_params[MAX_JOINTS];
  openarm::damiao_motor::StateResult states[MAX_JOINTS];

  // Get initial time for RT-safe periodic timing
  struct timespec next_wakeup;
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  const long cycle_ns = thread_cycle_time_.count() * 1000;  // Convert us to ns

  while (thread_running_) {
    struct timespec cycle_start;
    clock_gettime(CLOCK_MONOTONIC, &cycle_start);

    // Read commands from buffer
    int read_idx = cmd_read_idx_.load(std::memory_order_acquire);

    for (size_t i = 0; i < num_joints_; ++i) {
      const auto& cmd = cmd_buffers_[read_idx][i];
      mit_params[i] = {cmd.kp, cmd.kd, cmd.position, cmd.velocity, cmd.torque};
    }

    // Send MIT commands (RT-safe, non-blocking)
    ssize_t sent = openarm_rt_->send_mit_batch_rt(mit_params, num_joints_, 500);
    if (sent < 0) {
      static auto last_error_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArm_v10RTHW_Thread"),
                     "Failed to send MIT commands");
        last_error_time = now;
      }
    }

    // Receive motor states (RT-safe, non-blocking)
    ssize_t received =
        openarm_rt_->receive_states_batch_rt(states, MAX_JOINTS, 500);

    // Write states to buffer
    int write_idx = state_write_idx_.load(std::memory_order_acquire);

    for (size_t i = 0; i < num_joints_; ++i) {
      if (i < static_cast<size_t>(received) && states[i].valid) {
        state_buffers_[write_idx][i].position = states[i].position;
        state_buffers_[write_idx][i].velocity = states[i].velocity;
        state_buffers_[write_idx][i].torque = states[i].torque;
        state_buffers_[write_idx][i].valid = true;
        state_buffers_[write_idx][i].error_code = states[i].error_code;
      } else {
        state_buffers_[write_idx][i].valid = false;
      }
    }

    // Calculate next wakeup time (RT-safe periodic timing)
    next_wakeup.tv_nsec += cycle_ns;
    while (next_wakeup.tv_nsec >= 1000000000) {
      next_wakeup.tv_nsec -= 1000000000;
      next_wakeup.tv_sec++;
    }

    // RT-safe absolute time sleep
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);

    // Check for cycle overrun
    struct timespec cycle_end;
    clock_gettime(CLOCK_MONOTONIC, &cycle_end);

    long cycle_duration_ns =
        (cycle_end.tv_sec - cycle_start.tv_sec) * 1000000000L +
        (cycle_end.tv_nsec - cycle_start.tv_nsec);
    long cycle_duration_us = cycle_duration_ns / 1000;

    if (cycle_duration_us > thread_cycle_time_.count()) {
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 5000) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArm_v10RTHW_Thread"),
                    "CAN thread cycle overrun: %ld us (target: %ld us)",
                    cycle_duration_us, thread_cycle_time_.count());
        last_warn_time = now;
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArm_v10RTHW_Thread"),
              "CAN thread stopped");
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArm_v10RTHW,
                       hardware_interface::SystemInterface)
