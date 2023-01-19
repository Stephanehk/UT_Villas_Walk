// This file is based on UNSW Sydney's codebase, but has been modified significantly.
// Both copyright notices are provided below.
//
// Copyright (c) 2018 UNSW Sydney.  All rights reserved.
//
// Licensed under Team rUNSWift's original license. See the "LICENSE-runswift"
// file to obtain a copy of the license.
//
// ---------------------------------------------------------------------------------
//
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


#include "walk/walk.hpp"
#include "walk/maths_functions.hpp"
#include "std_msgs/msg/string.hpp"


#define STAND_HIP_HEIGHT 0.248
#define WALK_HIP_HEIGHT 0.23
#define CROUCH_STAND_PERIOD 0.5
// #define BASE_WALK_PERIOD 0.25
// #define BASE_LEG_LIFT 0.012


Walk::Walk(
  std::function<void(void)> notifyWalkDone,
  std::function<void(biped_interfaces::msg::SolePoses)> sendSolePoses,
  rclcpp::Node* walkNode)
: notifyWalkDone(notifyWalkDone), sendSolePoses(sendSolePoses), walkNode(walkNode)
{
  
}

void Walk::start(walk_msg::msg::Walk walk_command)
{
  
  

}

void Walk::notifyJoints(walk_sensor_msg::msg::Sensor sensor_readings)
{
  
  
}
