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

#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <optional>
#include <string>
#include <vector>

namespace openarm_hardware {

// Based on DM motor control modes
enum class ControlMode {
  UNINITIALIZED = -1,
  MIT = 1,                // MIT mode (torque/impedance control)
  POSITION_VELOCITY = 2,  // Position-Velocity mode (0x100 + ID frame)
  VELOCITY = 3,           // Velocity mode (not implemented)
  TORQUE_POSITION = 4     // Torque-Position mode (not implemented)
};

// Configuration for a single motor
struct MotorConfig {
  std::string name;
  openarm::damiao_motor::MotorType type;
  uint32_t send_can_id;
  uint32_t recv_can_id;
  double kp;            // Used in MIT mode
  double kd;            // Used in MIT mode
  double max_velocity;  // Max velocity for position mode (rad/s)
};

// Similar to motor config, but with a few extra parameters to convert between
// 0,1 joint states to motor radians
struct GripperConfig {
  std::string name;
  openarm::damiao_motor::MotorType motor_type;
  uint32_t send_can_id;
  uint32_t recv_can_id;
  double kp;
  double kd;
  double closed_position;
  double open_position;
  double motor_closed_radians;
  double motor_open_radians;
  double max_velocity;  // Max velocity for position mode

  // Maps joint [0, 1] to motor radians
  double to_radians(double joint_value) const;

  // Maps motor radians to joint [0, 1]
  double to_joint(double motor_radians) const;
};

struct ControllerConfig {
  // Which interface to connect to.
  std::string can_iface;

  // Use CAN FD instead of standard CAN.
  bool can_fd;

  // Configuration storage using POD types
  std::vector<MotorConfig> arm_joints;
  std::optional<GripperConfig> gripper_joint;
};

}  // namespace openarm_hardware
