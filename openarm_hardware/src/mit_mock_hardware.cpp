#include "openarm_hardware/mit_mock_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace openarm_hardware {

MITMockHardware::MITMockHardware() = default;

hardware_interface::CallbackReturn MITMockHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize joint data for each joint in the URDF
  joints_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    joints_[i].name = joint.name;

    // Parse optional parameters
    auto it = joint.parameters.find("initial_position");
    if (it != joint.parameters.end()) {
      joints_[i].position = std::stod(it->second);
      joints_[i].position_cmd = joints_[i].position;
    }

    it = joint.parameters.find("max_effort");
    if (it != joint.parameters.end()) {
      joints_[i].max_effort = std::stod(it->second);
    }

    it = joint.parameters.find("damping");
    if (it != joint.parameters.end()) {
      joints_[i].damping = std::stod(it->second);
    }

    RCLCPP_INFO(rclcpp::get_logger("MITMockHardware"),
                "Initialized mock joint '%s': pos=%.3f, max_effort=%.1f, "
                "damping=%.3f",
                joints_[i].name.c_str(), joints_[i].position,
                joints_[i].max_effort, joints_[i].damping);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MITMockHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("MITMockHardware"),
              "Configuring MITMockHardware with %zu joints", joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MITMockHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joints_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joints_[i].name, hardware_interface::HW_IF_POSITION,
        &joints_[i].position));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joints_[i].name, hardware_interface::HW_IF_VELOCITY,
        &joints_[i].velocity));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joints_[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MITMockHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, hardware_interface::HW_IF_POSITION,
        &joints_[i].position_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, hardware_interface::HW_IF_VELOCITY,
        &joints_[i].velocity_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, hardware_interface::HW_IF_EFFORT,
        &joints_[i].effort_cmd));

    // MIT-specific command interfaces
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, "kp", &joints_[i].kp_cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joints_[i].name, "kd", &joints_[i].kd_cmd));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MITMockHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("MITMockHardware"),
              "Activating MITMockHardware");

  // Initialize command interfaces to current state
  for (auto& joint : joints_) {
    joint.position_cmd = joint.position;
    joint.velocity_cmd = 0.0;
    joint.effort_cmd = 0.0;
    joint.kp_cmd = 0.0;
    joint.kd_cmd = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MITMockHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("MITMockHardware"),
              "Deactivating MITMockHardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MITMockHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // State is updated in write(), so read() is a no-op for this mock
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MITMockHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();

  if (dt <= 0.0 || dt > 1.0) {
    // Skip unrealistic time steps
    return hardware_interface::return_type::OK;
  }

  // Simulate MIT motor control for each joint
  for (auto& joint : joints_) {
    // Compute position error
    const double position_error = joint.position_cmd - joint.position;

    // Compute velocity error
    const double velocity_error = joint.velocity_cmd - joint.velocity;

    // MIT control law: torque = kp * pos_error + kd * vel_error + feedforward
    double computed_effort = joint.kp_cmd * position_error +
                             joint.kd_cmd * velocity_error + joint.effort_cmd;

    // Apply internal damping for stability
    computed_effort -= joint.damping * joint.velocity;

    // Clamp effort to limits
    computed_effort =
        std::clamp(computed_effort, -joint.max_effort, joint.max_effort);

    // Store actual effort
    joint.effort = computed_effort;

    // Simple Euler integration: assume unit inertia (torque = acceleration)
    // acceleration = effort / inertia (assume inertia = 1.0)
    const double acceleration = computed_effort;

    // Update velocity
    joint.velocity += acceleration * dt;

    // Update position
    joint.position += joint.velocity * dt;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::MITMockHardware,
                       hardware_interface::SystemInterface)
