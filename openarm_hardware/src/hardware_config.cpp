
#include "openarm_hardware/hardware_config.hpp"

namespace openarm_hardware {

std::string error_code_to_string(uint8_t error_code) {
  switch (error_code) {
    case 0x1:
      return "No error";
    case 0x8:
      return "Overvoltage";
    case 0x9:
      return "Undervoltage";
    case 0xA:
      return "Overcurrent";
    case 0xB:
      return "MOS overtemperature";
    case 0xC:
      return "Motor coil overtemperature";
    case 0xD:
      return "Communication loss";
    case 0xE:
      return "Overload";
    default:
      return "Unknown error (0x" + std::to_string(error_code) + ")";
  }
}

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
