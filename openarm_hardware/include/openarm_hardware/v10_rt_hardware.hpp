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
#include <atomic>
#include <memory>
#include <openarm/realtime/openarm.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "openarm_hardware/hardware_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace openarm_hardware {

// Maximum supported joints for pre-allocation
constexpr size_t MAX_JOINTS = 10;

/**
 * @brief RT-safe OpenArm V10 Hardware Interface
 *
 * This implementation addresses the main realtime violations:
 * - Pre-allocates all buffers
 * - Uses non-blocking CAN operations with timeouts
 * - Moves mode switching to async handler
 * - Uses RealtimeBuffer for thread-safe data exchange
 * - Replaces console logging with RT-safe throttled logging
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

  // Command mode switching
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

 public:
  // Control modes
  enum class ControlMode { UNINITIALIZED, POSITION_VELOCITY, MIT };

 private:

  // Configuration
  HardwareConfig config_;
  std::string motor_config_file_;
  std::vector<std::string> joint_names_;
  size_t num_joints_{0};
  std::vector<MotorConfig> motor_configs_;  // Motor configurations from YAML
  std::optional<GripperConfig> gripper_config_;  // Optional gripper configuration

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

  // RT-safe data exchange structures
  struct CommandData {
    std::array<double, MAX_JOINTS> positions;
    std::array<double, MAX_JOINTS> velocities;
    std::array<double, MAX_JOINTS> torques;
    ControlMode mode;
    bool valid;
  };

  struct StateData {
    std::array<double, MAX_JOINTS> positions;
    std::array<double, MAX_JOINTS> velocities;
    std::array<double, MAX_JOINTS> torques;
    std::array<uint8_t, MAX_JOINTS> error_codes;
    bool valid;
  };

  // Realtime buffers for thread-safe data exchange
  realtime_tools::RealtimeBuffer<CommandData> command_buffer_;
  realtime_tools::RealtimeBuffer<StateData> state_buffer_;

  // Flag to request mode switch from non-RT thread
  std::atomic<bool> mode_switch_requested_{false};

  // Control mode management
  std::atomic<ControlMode> current_mode_{ControlMode::UNINITIALIZED};
  std::atomic<ControlMode> pending_mode_{ControlMode::UNINITIALIZED};

  // Interface claiming states
  std::atomic<bool> position_interface_claimed_{false};
  std::atomic<bool> velocity_interface_claimed_{false};
  std::atomic<bool> effort_interface_claimed_{false};

  // RT-safe OpenArm interface
  std::unique_ptr<openarm::realtime::OpenArm> openarm_rt_;

  // Non-RT worker thread for CAN communication
  std::thread can_worker_thread_;
  std::atomic<bool> worker_running_{false};

  // RT-safe logging (using throttled macros)
  static constexpr int LOG_THROTTLE_MS = 1000;  // Log at most once per second

  // Helper methods
  bool parse_config(const hardware_interface::HardwareInfo& info);
  bool load_motor_config_from_yaml(const std::string& file_path);
  bool generate_joint_names();

  // CAN worker thread function
  void can_worker_loop();

  // Mode switching helpers (called from async handler)
  bool perform_mode_switch_async();
  bool switch_to_mit_mode();
  bool switch_to_position_mode();

  // RT-safe command/state helpers
  bool send_commands_rt();
  bool receive_states_rt();

  // Determine control mode from interfaces
  ControlMode determine_mode_from_interfaces(
      const std::vector<std::string>& interfaces);
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__V10_RT_HARDWARE_HPP_
