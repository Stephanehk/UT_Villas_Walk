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


#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "walk/walk.hpp"
#include "walk/BodyModel.hpp"
#include "walk/JointValues.hpp"
#include "walk/Sensors.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
// #include "biped_interfaces/msg/sole_poses.hpp"
#include "motion_interfaces/msg/kick.hpp"
#include "walk_msg/msg/walk.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"


using namespace std::placeholders;

void make_joint_msgs(const JointValues &joints, nao_command_msgs::msg::JointPositions &joint_angles,  nao_command_msgs::msg::JointStiffnesses &joint_stiffness)
{
    for(int i = 0; i < Joints::NUMBER_OF_JOINTS; i++)
    {
        joint_stiffness.indexes.push_back(i);
        joint_stiffness.stiffnesses.push_back(joints.stiffnesses[i]);
    }

    for(int i = Joints::LShoulderPitch; i <= Joints::RWristYaw; i++)
    {
        joint_angles.indexes.push_back(i);
        joint_angles.positions.push_back(joints.angles[i]);
    }
}

class WalkNode : public rclcpp::Node
{
public:
  WalkNode()
  : Node("WalkNode"), sensor_values(true)
  {
    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        
        sensor_values.populate_joint_positions(*sensor_joints);

        if (this->started_walk)
        {
          JointValues j = walk.notifyJoints(action_command, sensor_values, bodyModel);
          make_joint_msgs(j, joint_angles, joint_stiffness);

          pub_joint_angles->publish(joint_angles);
          pub_joint_stiffness->publish(joint_stiffness);
        }
        
      });

    sub_gyroscope = create_subscription<nao_sensor_msgs::msg::Gyroscope>(
      "sensors/gyroscope", 1,
      [this](nao_sensor_msgs::msg::Gyroscope::SharedPtr gyr) {
   
         //RCLCPP_INFO(this->get_logger(), "Recived gyroscope reading, x=%f, y = %f, z=%f\n",gyr->x, gyr->y, gyr->z);
        sensor_values.populate_gyro(*gyr);

      }
    );

    sub_angle = create_subscription<nao_sensor_msgs::msg::Angle>(
      "sensors/angle", 1,
      [this](nao_sensor_msgs::msg::Angle::SharedPtr angle) {
       
         //RCLCPP_INFO(this->get_logger(), "Recived gyroscope reading, x=%f, y = %f, z=%f\n",gyr->x, gyr->y, gyr->z);
        sensor_values.populate_angles(*angle);

      }
    );

    sub_fsr = create_subscription<nao_sensor_msgs::msg::FSR>(
      "sensors/fsr", 1,
      [this](nao_sensor_msgs::msg::FSR::SharedPtr fsr) {
   
        sensor_values.populate_fsr(*fsr);

      }
    );
    
    
    sub_walk_start =
      create_subscription<walk_msg::msg::Walk>(
      "motion/walk", 10,
      [this](walk_msg::msg::Walk::SharedPtr walk_command) {
        this->started_walk = true;
        action_command.make_from_walk_command(*walk_command);
        walk.start();
      });

    // pub_sole_poses = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);

    // pub_walk_done = create_publisher<std_msgs::msg::Empty>("motion/walk_done", 1);

    pub_joint_angles = create_publisher<nao_command_msgs::msg::JointPositions>("effectors/joint_positions", 1);
    pub_joint_stiffness = create_publisher<nao_command_msgs::msg::JointStiffnesses>("effectors/joint_stiffness", 1);

  }


private:

  Walk walk;
  ActionCommand action_command;
  SensorValues sensor_values;
  BodyModel bodyModel;

  nao_command_msgs::msg::JointPositions joint_angles;
  nao_command_msgs::msg::JointStiffnesses joint_stiffness;
  

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<nao_sensor_msgs::msg::Gyroscope>::SharedPtr sub_gyroscope;
  rclcpp::Subscription<nao_sensor_msgs::msg::Angle>::SharedPtr sub_angle;
  rclcpp::Subscription<nao_sensor_msgs::msg::FSR>::SharedPtr sub_fsr;
  rclcpp::Subscription<walk_msg::msg::Walk>::SharedPtr sub_walk_start;

  rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_angles;
  rclcpp::Publisher<nao_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffness;
  //rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_states;

  //walk_msg::msg::Walk walk_command;
  bool started_walk=false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
