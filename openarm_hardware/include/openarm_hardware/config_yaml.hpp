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

#include <yaml-cpp/yaml.h>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include "hardware_config.hpp"

namespace openarm_hardware {

inline openarm::damiao_motor::MotorType parse_motor_type(const std::string& type_str) {
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

}  // namespace openarm_hardware

// YAML conversion implementations
namespace YAML {

template <>
struct convert<openarm_hardware::MotorConfig> {
  static bool decode(const Node& node, openarm_hardware::MotorConfig& config) {
    config.name = node["name"].as<std::string>();
    config.type = openarm_hardware::parse_motor_type(node["type"].as<std::string>());
    config.send_can_id = node["send_can_id"].as<uint32_t>();
    config.recv_can_id = node["recv_can_id"].as<uint32_t>();
    config.kp = node["kp"].as<double>(0.0);
    config.kd = node["kd"].as<double>(0.0);
    config.max_velocity = node["max_velocity"].as<double>(10.0);  // Default 10 rad/s
    return true;
  }
};

template <>
struct convert<openarm_hardware::GripperConfig> {
  static bool decode(const Node& node, openarm_hardware::GripperConfig& config) {
    config.name = node["name"].as<std::string>();
    config.motor_type = openarm_hardware::parse_motor_type(node["motor_type"].as<std::string>());
    config.send_can_id = node["send_can_id"].as<uint32_t>();
    config.recv_can_id = node["recv_can_id"].as<uint32_t>();
    config.kp = node["kp"].as<double>(0.0);
    config.kd = node["kd"].as<double>(0.0);
    config.closed_position = node["closed_position"].as<double>();
    config.open_position = node["open_position"].as<double>();
    config.motor_closed_radians = node["motor_closed_radians"].as<double>();
    config.motor_open_radians = node["motor_open_radians"].as<double>();
    config.max_velocity = node["max_velocity"].as<double>(5.0);  // Default 5 rad/s
    return true;
  }
};

template <>
struct convert<openarm_hardware::ControllerConfig> {
  static bool decode(const Node& node, openarm_hardware::ControllerConfig& config) {
    config.can_iface = node["can_iface"].as<std::string>();
    config.can_fd = node["can_fd"].as<bool>(false);
    config.enable_csv_logging = node["enable_csv_logging"].as<bool>(false);

    if (node["arm"]) {
      for (const auto& motor_node : node["arm"]) {
        config.arm_joints.push_back(motor_node.as<openarm_hardware::MotorConfig>());
      }
    }

    if (node["gripper"]) {
      config.gripper_joint = node["gripper"].as<openarm_hardware::GripperConfig>();
    }

    return true;
  }
};

}  // namespace YAML
