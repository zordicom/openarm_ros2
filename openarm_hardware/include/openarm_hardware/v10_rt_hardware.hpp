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

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm/realtime/can.hpp"
#include "openarm/realtime/canfd.hpp"
#include "openarm/realtime/openarm.hpp"
#include "openarm_hardware/hardware_config.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

/**
 * @brief RT-Safe OpenArm V10 Hardware Interface with Background CAN Thread
 *
 * This implementation decouples CAN bus I/O from the ros2_control cycle by
 * running a background thread that continuously reads and writes to the CAN
 * interface. The controller's read()/write() methods use lock-free double
 * buffering to exchange data with the background thread, eliminating jitter
 * from CAN operations.
 *
 * Key Features:
 * - Background thread for continuous CAN I/O
 * - Lock-free double buffering with atomic swaps
 * - RT-safe controller loop (no blocking operations)
 * - Uses openarm::realtime::OpenArm (RT-safe API)
 * - Configurable thread priority and cycle time
 */
class OpenArm_v10RTHW : public hardware_interface::SystemInterface {
 public:
  OpenArm_v10RTHW();
  ~OpenArm_v10RTHW();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Double-buffered state for lock-free communication
  struct MotorState {
    double position;
    double velocity;
    double torque;
    bool valid;
    uint8_t error_code;
  };

  struct MotorCommand {
    double position;
    double velocity;
    double torque;
    double kp;
    double kd;
  };

  // Per-joint state and command buffers (double buffered)
  static constexpr size_t MAX_JOINTS = 15;
  std::array<MotorState, MAX_JOINTS> state_buffers_[2];
  std::array<MotorCommand, MAX_JOINTS> cmd_buffers_[2];

  // Atomic indices for lock-free buffer swapping
  std::atomic<int> state_read_idx_{0};  // Controller reads from this buffer
  std::atomic<int> state_write_idx_{
      1};  // Background thread writes to this buffer
  std::atomic<int> cmd_read_idx_{
      1};  // Background thread reads from this buffer
  std::atomic<int> cmd_write_idx_{0};  // Controller writes to this buffer

  // Background thread control
  std::atomic<bool> thread_running_{false};
  std::unique_ptr<std::thread> can_thread_;
  std::chrono::microseconds thread_cycle_time_{1000};  // 1ms default

  // RT-safe OpenArm instance
  std::unique_ptr<openarm::realtime::OpenArm> openarm_rt_;

  // ROS2 control joint names and vectors (exposed to controllers)
  std::array<std::string, MAX_JOINTS> joint_names_;
  std::array<double, MAX_JOINTS> pos_commands_;
  std::array<double, MAX_JOINTS> vel_commands_;
  std::array<double, MAX_JOINTS> tau_commands_;
  std::array<double, MAX_JOINTS> kp_commands_;
  std::array<double, MAX_JOINTS> kd_commands_;
  std::array<double, MAX_JOINTS> pos_states_;
  std::array<double, MAX_JOINTS> vel_states_;
  std::array<double, MAX_JOINTS> tau_states_;

  // Configuration
  ControllerConfig config_;
  HardwareConfig hw_config_;
  size_t num_joints_{0};
  int thread_priority_{80};  // RT priority for CAN thread

  // Helper methods
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void can_thread_loop();
  void swap_state_buffers();
  void swap_command_buffers();
  bool check_motor_errors();
};

}  // namespace openarm_hardware
