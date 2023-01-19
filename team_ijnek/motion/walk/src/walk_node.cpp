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
#include "std_msgs/msg/empty.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "biped_interfaces/msg/sole_poses.hpp"
#include "motion_interfaces/msg/kick.hpp"
#include "walk_msg/msg/walk.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"


using namespace std::placeholders;

class WalkNode : public rclcpp::Node
{
public:
  WalkNode()
  : Node("WalkNode"),
    walk(
      std::bind(&WalkNode::notifyWalkDone, this),
      std::bind(&WalkNode::sendSolePoses, this, _1),
      this)
  {
    

    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {
        
        this->sensor_readings.lkneepitch = sensor_joints->positions[10];
        this->sensor_readings.rkneepitch = sensor_joints->positions[15];
        this->sensor_readings.lhiproll = sensor_joints->positions[8];
        this->sensor_readings.rhiproll = sensor_joints->positions[13];

        if (this->started_walk){
          walk.notifyJoints(this->sensor_readings);
        }
        
      });

    sub_gyroscope = create_subscription<nao_sensor_msgs::msg::Gyroscope>(
      "sensors/gyroscope", 1,
      [this](nao_sensor_msgs::msg::Gyroscope::SharedPtr gyr) {
   
         //RCLCPP_INFO(this->get_logger(), "Recived gyroscope reading, x=%f, y = %f, z=%f\n",gyr->x, gyr->y, gyr->z);
        this->sensor_readings.gyrx = gyr->x;
        this->sensor_readings.gyry = gyr->y;
        this->sensor_readings.gyrz = gyr->z;
      }
    );

    sub_angle = create_subscription<nao_sensor_msgs::msg::Angle>(
      "sensors/angle", 1,
      [this](nao_sensor_msgs::msg::Angle::SharedPtr angle) {
       
         //RCLCPP_INFO(this->get_logger(), "Recived gyroscope reading, x=%f, y = %f, z=%f\n",gyr->x, gyr->y, gyr->z);
        this->sensor_readings.anglex = angle->x;
        this->sensor_readings.angley = angle->y;
      }
    );

    sub_fsr = create_subscription<nao_sensor_msgs::msg::FSR>(
      "sensors/fsr", 1,
      [this](nao_sensor_msgs::msg::FSR::SharedPtr fsr) {
   
        //this->last_recieved_fsr = *fsr;
        this->sensor_readings.lfsrfl = fsr->l_foot_front_left;
        this->sensor_readings.lfsrfr = fsr->l_foot_front_right;
        this->sensor_readings.lfsrrl = fsr->l_foot_back_left;
        this->sensor_readings.lfsrrr = fsr->l_foot_back_right;

        this->sensor_readings.rfsrfl = fsr->r_foot_front_left;
        this->sensor_readings.rfsrfr = fsr->r_foot_front_right;
        this->sensor_readings.rfsrrl = fsr->r_foot_back_left;
        this->sensor_readings.rfsrrr = fsr->r_foot_back_right;
      }
    );
    
    
    sub_walk_start =
      create_subscription<walk_msg::msg::Walk>(
      "motion/walk", 10,
      [this](walk_msg::msg::Walk::SharedPtr walk_command) {
        this->started_walk = true;
        walk.start(*walk_command);
      });

    pub_sole_poses = create_publisher<biped_interfaces::msg::SolePoses>("motion/sole_poses", 1);

    pub_walk_done = create_publisher<std_msgs::msg::Empty>("motion/walk_done", 1);

  }


private:
  Walk walk;

  void notifyWalkDone()
  {
    pub_walk_done->publish(std_msgs::msg::Empty{});
  }

  void sendSolePoses(biped_interfaces::msg::SolePoses sole_poses)
  {
    pub_sole_poses->publish(sole_poses);

    //nao_command_msgs::msg::JointPositions joint_state;
    // joint_state.indexes.push_back(nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH);
    // joint_state.positions.push_back(0);

    //pub_joint_states->publish(joint_state);
  }

  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Subscription<nao_sensor_msgs::msg::Gyroscope>::SharedPtr sub_gyroscope;
  rclcpp::Subscription<nao_sensor_msgs::msg::Angle>::SharedPtr sub_angle;
  rclcpp::Subscription<nao_sensor_msgs::msg::FSR>::SharedPtr sub_fsr;
  rclcpp::Subscription<walk_msg::msg::Walk>::SharedPtr sub_walk_start;

  rclcpp::Publisher<biped_interfaces::msg::SolePoses>::SharedPtr pub_sole_poses;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_walk_done;
  //rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_states;

  //walk_msg::msg::Walk walk_command;
  walk_sensor_msg::msg::Sensor sensor_readings;
  bool started_walk=false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
