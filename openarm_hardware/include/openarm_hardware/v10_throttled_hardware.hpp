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

#ifndef OPENARM_HARDWARE__V10_THROTTLED_HARDWARE_HPP_
#define OPENARM_HARDWARE__V10_THROTTLED_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <openarm/realtime/openarm.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "openarm_hardware/hardware_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

// Maximum supported joints for pre-allocation
constexpr size_t MAX_JOINTS = 10;

/**
 * @brief Simplified throttled OpenArm V10 Hardware Interface
 *
 * Key design principles:
 * - No worker threads - everything happens in read()/write() calls
 * - write() throttles CAN writes to prevent bus overload (~312 Hz)
 * - read() always tries to read from CAN (non-blocking, no bus traffic)
 * - Controller runs at high frequency (e.g. 1000 Hz)
 * - Latest motor states cached and returned to controller at every read()
 */
class OpenArm_v10ThrottledHardware : public hardware_interface::SystemInterface {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  OpenArm_v10ThrottledHardware();
  ~OpenArm_v10ThrottledHardware() = default;

  // Hardware Interface lifecycle methods
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // Export interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // RT-safe read/write
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // Command mode switching
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

 private:
  // Configuration
  HardwareConfig config_;
  ControllerConfig controller_config_;
  std::vector<std::string> joint_names_;
  size_t num_joints_{0};

  // Pre-allocated state and command buffers
  std::array<double, MAX_JOINTS> pos_states_{};
  std::array<double, MAX_JOINTS> vel_states_{};
  std::array<double, MAX_JOINTS> tau_states_{};
  std::array<double, MAX_JOINTS> pos_commands_{};
  std::array<double, MAX_JOINTS> vel_commands_{};
  std::array<double, MAX_JOINTS> tau_commands_{};

  // Pre-allocated CAN command buffers
  std::array<openarm::damiao_motor::MITParam, MAX_JOINTS> mit_params_{};
  std::array<openarm::damiao_motor::PosVelParam, MAX_JOINTS> posvel_params_{};

  // Control mode management
  ControlMode current_mode_{ControlMode::UNINITIALIZED};

  // Interface claiming state
  bool gripper_claimed_{false};

  // RT-safe OpenArm interface
  std::unique_ptr<openarm::realtime::OpenArm> openarm_rt_;

  // Throttling for CAN writes (reads are always non-blocking)
  std::chrono::steady_clock::time_point last_can_write_;
  static constexpr int64_t CAN_WRITE_INTERVAL_US = 3200;  // 312.5 Hz

  // Performance stats
  struct Stats {
    uint64_t read_count{0};
    uint64_t write_count{0};
    uint64_t can_writes{0};
    uint64_t can_reads{0};
    uint64_t tx_skipped{0};     // Write cycles skipped due to throttling
    uint64_t rx_no_data{0};     // Read cycles with no data available
    uint64_t rx_received{0};    // Successful motor state reads
  };
  Stats stats_;

  // Stats logging
  std::chrono::steady_clock::time_point last_stats_log_;
  static constexpr int STATS_LOG_INTERVAL_SEC = 10;

  // Helper methods
  bool parse_config(const hardware_interface::HardwareInfo& info);
  bool switch_to_mit_mode();
  bool switch_to_position_mode();
  ControlMode determine_mode_from_interfaces(
      const std::vector<std::string>& interfaces);
  void log_stats();
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__V10_THROTTLED_HARDWARE_HPP_
