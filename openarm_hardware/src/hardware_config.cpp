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

#include "openarm_hardware/hardware_config.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace openarm_hardware {

double gripper_joint_to_motor_radians(const GripperConfig& config,
                                      double joint_value) {
  // Linear interpolation: joint [0, 1] -> motor [closed, open] radians
  return config.motor_closed_radians +
         joint_value * (config.motor_open_radians - config.motor_closed_radians);
}

double gripper_motor_radians_to_joint(const GripperConfig& config,
                                      double motor_radians) {
  // Inverse mapping: motor radians -> joint [0, 1]
  double range = config.motor_open_radians - config.motor_closed_radians;
  if (std::abs(range) < 1e-6) {
    return 0.0;  // Avoid division by zero
  }
  return (motor_radians - config.motor_closed_radians) / range;
}

double GripperConfig::to_radians(double joint_value) const {
  return motor_closed_radians +
         joint_value * (motor_open_radians - motor_closed_radians);
}

double GripperConfig::to_joint(double motor_radians) const {
  double range = motor_open_radians - motor_closed_radians;
  if (std::abs(range) < 1e-6) {
    return 0.0;  // Avoid division by zero
  }
  return (motor_radians - motor_closed_radians) / range;
}

openarm::damiao_motor::MotorType parse_motor_type_param(const std::string& type_str) {
  if (type_str == "DM3507") return openarm::damiao_motor::MotorType::DM3507;
  if (type_str == "DM4310") return openarm::damiao_motor::MotorType::DM4310;
  if (type_str == "DM4310_48V") return openarm::damiao_motor::MotorType::DM4310_48V;
  if (type_str == "DM4340") return openarm::damiao_motor::MotorType::DM4340;
  if (type_str == "DM4340_48V") return openarm::damiao_motor::MotorType::DM4340_48V;
  if (type_str == "DM6006") return openarm::damiao_motor::MotorType::DM6006;
  if (type_str == "DM8006") return openarm::damiao_motor::MotorType::DM8006;
  if (type_str == "DM8009") return openarm::damiao_motor::MotorType::DM8009;
  if (type_str == "DM10010L") return openarm::damiao_motor::MotorType::DM10010L;
  if (type_str == "DM10010") return openarm::damiao_motor::MotorType::DM10010;
  if (type_str == "DMH3510") return openarm::damiao_motor::MotorType::DMH3510;
  if (type_str == "DMH6215") return openarm::damiao_motor::MotorType::DMH6215;
  if (type_str == "DMG6220") return openarm::damiao_motor::MotorType::DMG6220;

  throw std::runtime_error("Unknown motor type: " + type_str);
}

bool parse_bool_param(const std::string& str) {
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (lower == "true" || lower == "1" || lower == "yes") {
    return true;
  }
  if (lower == "false" || lower == "0" || lower == "no") {
    return false;
  }

  throw std::runtime_error("Invalid boolean value: " + str);
}

std::string error_code_to_string(uint8_t error_code) {
  switch (error_code) {
    case 0x00:
      return "No error";
    case 0x01:
      return "Over temperature";
    case 0x02:
      return "Over current";
    case 0x04:
      return "Over voltage";
    case 0x08:
      return "Encoder error";
    case 0x10:
      return "Phase current unbalance";
    default:
      return "Unknown error";
  }
}

}  // namespace openarm_hardware
