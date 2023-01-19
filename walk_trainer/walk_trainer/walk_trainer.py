# Copyright 2021 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,LogInfo,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import rclpy
from rclpy.node import Node as rclpy_Node
from billiard import Process
import time
import psutil
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import math
import optuna


from launch_ros.actions import Node
#from walk_msg.msg import Walk
import walk_msg.msg
import nao_sensor_msgs.msg
import soccer_object_msgs.msg
from . import var_tracker
import rcss3d_controller_msgs.msg
import geometry_msgs.msg
#"rcss3d_controller_msgs/msg/joint_command.hpp" 

class MinimalPublisher(rclpy_Node):

    def __init__(self,msg):
        super().__init__('minimal_publisher')

        # qos = QoSProfile(
        # depth=10,
        # reliability=QoSReliabilityPolicy.RELIABLE,
        # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.publisher_ = self.create_publisher(walk_msg.msg.Walk, '/motion/walk',10)
        self.publisher_.publish(msg)
       
        self.get_logger().info('Publishing ')

    
    # def timer_callback(self):

    #     self.publisher_.publish(self.msg)
    #     self.get_logger().info('Publishing: "%s"' % self.msg.isready)
    #     self.i += 1

class MinimalSubscriber(rclpy_Node):
    def __init__(self, base_walk_period, twist):
        super().__init__("minimal_subscriber")

        qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE)
        
        self.ref_x = 0
        self.ref_y = 0 
        self.ref_theta = 0
        self.init_ref = False
        self.step = 0

        self.base_walk_period = base_walk_period
        self.twist = twist

        self.acc_subscriber = self.create_subscription(nao_sensor_msgs.msg.Accelerometer, "sensors/accelerometer", self.acc_listener,qos) 
        self.my_pos_subscriber = self.create_subscription(geometry_msgs.msg.Twist, "vision/mypos", self.robot_listener, qos)   
    
    def acc_listener(self,msg):
        #self.get_logger().info("Recived accelerometer reading, x=%f, y = %f, z=%f\n" % (msg.x, msg.y, msg.z))
        if (msg.z <= -10 and not var_tracker.fallen):
            var_tracker.ep_return -= 1000
            var_tracker.fallen = True

    def update_ref_pos(self):
        #update x and y based on desired motion, TODO on how to do this
        self.dt = self.base_walk_period*10
        self.ref_x += self.base_walk_period*((-self.twist.linear.x/self.dt)*math.cos(self.twist.angular.z) - (self.twist.linear.y/self.dt)*math.sin(self.twist.angular.z))
        self.ref_y += self.base_walk_period*((-self.twist.linear.x/self.dt)*math.sin(self.twist.angular.z) + (self.twist.linear.y/self.dt)*math.cos(self.twist.angular.z))
        self.ref_theta += self.dt*self.twist.angular.z

    def robot_listener(self,msg):
        if not self.init_ref:
            self.ref_x = msg.linear.x
            self.ref_y = msg.linear.y
            self.ref_theta = msg.angular.z
            self.init_ref = True
        else:
            self.update_ref_pos()
        
        # self.get_logger().info("Step %f, Robot is at %f, %f, %f\n" % (self.step, msg.linear.x, msg.linear.y, msg.angular.z)) 
        # self.get_logger().info("Step %f, Ref point is at %f, %f, %f\n\n" % (self.step, self.ref_x, self.ref_y, self.ref_theta)) 
        # self.step+=1
        d_err = math.sqrt((self.ref_x - msg.linear.x)**2 + (self.ref_y - msg.linear.y)**2)
        h_err = math.sqrt((self.ref_theta - msg.angular.z)**2)

        if not var_tracker.fallen:
            var_tracker.ep_return -= (d_err + h_err)
        
        #self.get_logger().info("Error %f, %f, %f \n" % (d_err, h_err, var_tracker.ep_return)) 



def generate_launch_description():
    

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('team', default_value='ijnek'),
        DeclareLaunchArgument('unum', default_value='2'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('theta', default_value='0.0'),
        Node(
            package='rcss3d_nao',
            executable='rcss3d_nao',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'team': LaunchConfiguration('team'),
                'unum': LaunchConfiguration('unum'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'theta': LaunchConfiguration('theta')
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nao_state_publisher'),
                '/launch', '/nao_state_publisher_launch.py']),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
            }.items(),
        ),
        Node(
            package='static_pose_publisher',
            executable='static_pose_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'theta': LaunchConfiguration('theta')
            }]
        ),
        Node(
            package='nao_ik',
            executable='ik_node',
            namespace=LaunchConfiguration('namespace'),
        ),
        Node(
            package='crouch',
            executable='crouch_node',
            namespace=LaunchConfiguration('namespace'),
        ),
        Node(
            package='kick',
            executable='kick_node',
            namespace=LaunchConfiguration('namespace'),
        ),
        Node(
            package='walk',
            executable='walk_node',
            namespace=LaunchConfiguration('namespace'),
        ),
        
    ])


def startLaunchServiceProcess(launchDesc):
    """Starts a Launch Service process. To be called from subclasses.
    Args:
         launchDesc : LaunchDescription obj.
    """
    # Create the LauchService and feed the LaunchDescription obj. to it.
    launchService = LaunchService()
    launchService.include_launch_description(launchDesc)
    process = Process(target=launchService.run)
    #The daemon process is terminated automatically before the main program exits,
    # to avoid leaving orphaned processes running
    process.daemon = True
    process.start()

    return process

def spinNodeInBg(node):
    process = Process(target = rclpy.spin,args=(node,))
    process.daemon = True
    process.start()
    return process

def kill_process(pid):
    parent = psutil.Process(pid)
    for child in parent.children(recursive=True):
        child.kill()
    parent.kill()

def execute_episode(trial):
    #This code is only for training with CMAES, now it is outdated
    var_tracker.ep_return = 0
    var_tracker.fallen = False

    rclpy.init()
    
    # #launch player
    player_process = startLaunchServiceProcess(generate_launch_description())

    time.sleep(5)

    fcoeff = trial.suggest_float("fcoeff", 0, 0.1)
    lcoeff = trial.suggest_float("lcoeff", 0, 0.1)
    basewalkperiod = trial.suggest_float("basewalkperiod", 0.1, 0.5)
    baseleglift = trial.suggest_float("baseleglift", 0, 0.1)

    anklecoeff = trial.suggest_float("anklecoeff", 0, 0.1)
    #rotcoeff = trial.suggest_float("rotcoeff", 0, 1)

    ankleoff = trial.suggest_float("ankleoff", 0, 0.1)
    #rotoff = trial.suggest_float("rotoff", 0, 1)

    msg = walk_msg.msg.Walk()
        
    msg.fcoeff = fcoeff
    msg.lcoeff = lcoeff
    msg.isready = 1
    msg.basewalkperiod = basewalkperiod
    msg.baseleglift = baseleglift
    msg.twist.linear.x = -0.13 #UNTESTED, CHABGE BACK TO -0.08
    msg.twist.angular.z = 0.0 

    #controls ankle pitch (for balance)
    msg.anklecoeff = anklecoeff
    msg.ankleoff = ankleoff

    #controls angle yaw (for turning)
    msg.rotoff = 0.0
    msg.rotcoeff = 0.0
    msg.twist.angular.y = 0.0

    node = MinimalPublisher(msg)
    sensor_node = MinimalSubscriber(msg.basewalkperiod, msg.twist)

    executer = rclpy.executors.MultiThreadedExecutor()
    executer.add_node(node)
    executer.add_node(sensor_node)

    for step in range(1000):
        executer.spin_once()
        time.sleep(0.02)
        if var_tracker.fallen:
            break

    kill_process(player_process.pid)
    rclpy.shutdown()

    #print("EPISODE RETURN:", var_tracker.ep_return)
    f = open("episode_returns.txt", "a")
    f.write(str(var_tracker.ep_return) + ", ")
    f.close()
    return -var_tracker.ep_return

def train():
    f = open("episode_returns.txt", "w")
    f.write("")
    f.close()

    cma = optuna.samplers.CmaEsSampler()
    source_study = optuna.create_study(sampler=cma)
    source_study.optimize(execute_episode, n_trials=5000)

    
    print(
        f"Best value on the source task: {source_study.best_value},"
        f" (params: {source_study.best_params}\n"
    )

    f = open("episode_results.txt", "w")
    f.write("Best value on the source task: " + str(source_study.best_value))
    f.write("\n (params: " + str(source_study.best_params))
    f.close()

    n_episodes = 100
    for eps_i in range(n_episodes):
        execute_episode(eps_i)
        print("EPISODE RETURN:", var_tracker.ep_return)



def main():
   
    rclpy.init()
    
    # #launch player
    player_process = startLaunchServiceProcess(generate_launch_description())

    time.sleep(5)

    #send message with walk params
    msg = walk_msg.msg.Walk()
    

    node = MinimalPublisher(msg)
    executer = rclpy.executors.MultiThreadedExecutor()
    executer.add_node(node)
    executer.spin_once()

    kill_process(player_process.pid)
    rclpy.shutdown()



if __name__ == "__main__":
    main()