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

#ifndef OPENARM_HARDWARE__V10_RT_HARDWARE_HPP_
#define OPENARM_HARDWARE__V10_RT_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <openarm/realtime/openarm.hpp>
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
constexpr ssize_t MAX_JOINTS = 10;

/**
 * @brief RT-safe OpenArm V10 Hardware Interface
 *
 * Key design principles:
 * - No worker threads - everything happens in read()/write() calls
 * - write() sends CAN writes and reads responses in same cycle
 * - read() returns cached motor states
 * - Controller runs at high frequency (e.g. 1000 Hz)
 * - Latest motor states cached and returned to controller at every read()
 */
class OpenArm_v10RTHardware : public hardware_interface::SystemInterface {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  OpenArm_v10RTHardware();
  ~OpenArm_v10RTHardware() = default;

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


 private:
  // Configuration
  HardwareConfig config_;
  ControllerConfig controller_config_;
  std::vector<std::string> joint_names_;
  ssize_t num_joints_{0};

  // Pre-allocated state and command buffers
  std::array<double, MAX_JOINTS> pos_states_{};
  std::array<double, MAX_JOINTS> vel_states_{};
  std::array<double, MAX_JOINTS> tau_states_{};
  std::array<double, MAX_JOINTS> pos_commands_{};
  std::array<double, MAX_JOINTS> vel_commands_{};
  std::array<double, MAX_JOINTS> tau_commands_{};
  std::array<double, MAX_JOINTS> kp_commands_{};
  std::array<double, MAX_JOINTS> kd_commands_{};

  // Default kp/kd values from configuration (used when command interface not claimed)
  std::array<double, MAX_JOINTS> default_kp_{};
  std::array<double, MAX_JOINTS> default_kd_{};

  // Pre-allocated CAN command buffers
  std::array<openarm::damiao_motor::MITParam, MAX_JOINTS> mit_params_{};

  // Pre-allocated CAN read buffer
  std::array<openarm::damiao_motor::StateResult, MAX_JOINTS> motor_states_{};

  // RT-safe OpenArm interface
  std::unique_ptr<openarm::realtime::OpenArm> openarm_rt_;

  // Performance stats
  struct Stats {
    uint64_t read_count{0};
    uint64_t write_count{0};
    uint64_t can_writes{0};
    uint64_t can_reads{0};
    uint64_t tx_skipped{0};   // Write cycles skipped due to throttling
    uint64_t tx_partial{0};   // Partial writes (not all motors sent)
    uint64_t rx_no_data{0};   // Read cycles with no data available
    uint64_t rx_received{0};  // Successful motor state reads
    uint64_t rx_partial{0};   // Partial reads (not all motors received)

    // Per-motor tracking
    std::array<uint64_t, MAX_JOINTS>
        motor_sends{};  // Commands sent to each motor
    std::array<uint64_t, MAX_JOINTS>
        motor_receives{};  // States received from each motor
  };
  Stats stats_;

  // Stats logging
  std::chrono::steady_clock::time_point last_stats_log_;
  static constexpr int STATS_LOG_INTERVAL_SEC = 10;

  // Warning throttling (RT-safe)
  std::chrono::steady_clock::time_point last_partial_write_warn_;
  std::chrono::steady_clock::time_point last_partial_read_warn_;
  std::chrono::steady_clock::time_point last_no_data_warn_;
  static constexpr int64_t WARN_THROTTLE_MS = 1000;

  // Helper methods
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void log_stats();
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__V10_RT_HARDWARE_HPP_
