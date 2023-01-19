// Copyright 2021 Kenji Brameld
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

#ifndef WALK__WALK_HPP_
#define WALK__WALK_HPP_

#include <functional>
#include "biped_interfaces/msg/sole_poses.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "walk_msg/msg/walk.hpp"
#include "walk_sensor_msg/msg/sensor.hpp"



class Walk
{
public:
  Walk(
    std::function<void(void)> notifyWalkDone,
    std::function<void(biped_interfaces::msg::SolePoses)> sendSolePoses,
    rclcpp::Node* walkNode);
  void start(walk_msg::msg::Walk walk_command);
  void notifyJoints(walk_sensor_msg::msg::Sensor sensor_readings);

private:
  std::function<void(void)> notifyWalkDone;
  std::function<void(biped_interfaces::msg::SolePoses)> sendSolePoses;
  rclcpp::Node* walkNode;

  walk_msg::msg::Walk walk_command;
  walk_sensor_msg::msg::Sensor sensor_readings;
  

  
};

#endif  // KICK__KICK_HPP_
