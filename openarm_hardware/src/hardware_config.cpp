
#include "openarm_hardware/hardware_config.hpp"

namespace openarm_hardware {

double GripperConfig::to_radians(double joint_value) const {
  double range = open_position - closed_position;
  double motor_range = motor_open_radians - motor_closed_radians;

  return motor_closed_radians +
         ((joint_value - closed_position) / range) * motor_range;
}

double GripperConfig::to_joint(double motor_radians) const {
  double range = open_position - closed_position;
  double motor_range = motor_open_radians - motor_closed_radians;
  return closed_position +
         ((motor_radians - motor_closed_radians) / motor_range) * range;
}

}  // namespace openarm_hardware
