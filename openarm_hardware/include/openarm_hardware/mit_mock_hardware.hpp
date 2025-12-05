#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "openarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace openarm_hardware {

class MITMockHardware : public hardware_interface::SystemInterface {
 public:
  MITMockHardware();
  ~MITMockHardware() override = default;

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
  struct JointData {
    std::string name;

    // State interfaces
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};

    // Command interfaces
    double position_cmd{0.0};
    double velocity_cmd{0.0};
    double effort_cmd{0.0};
    double kp_cmd{0.0};
    double kd_cmd{0.0};

    // Simulation parameters
    double damping{0.1};      // Internal damping for stability
    double max_effort{10.0};  // Maximum torque/effort
  };

  std::vector<JointData> joints_;

  // Parse bool parameter helper
  bool parse_bool_param(const std::string& param_value) const {
    return param_value == "true" || param_value == "True" || param_value == "1";
  }
};

}  // namespace openarm_hardware
