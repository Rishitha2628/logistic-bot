#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import os
import yaml
import math
# from eyantra_warehouse.config import config.yaml
from rclpy.node import Node
from std_msgs.msg import Bool

from rclpy.duration import Duration
from ebot_docking.srv import DockSw
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory



"""
Basic navigation demo to go to pose.
"""

def euler_to_quaternion(yaw):
    """
    Convert Euler yaw angle (in radians) to quaternion (x, y, z, w) representation.
    """
    half_yaw = 0.5 * yaw
    sin_yaw = math.sin(half_yaw)
    cos_yaw = math.cos(half_yaw)

    # x = 0.0  # x component of quaternion
    # y = 0.0  # y component of quaternion
    z = sin_yaw  # z component of quaternion
    w = cos_yaw  # w component of quaternion

    return z, w


def main(args=None):
    rclpy.init(args=args)   
    node = rclpy.create_node('dock_sw_client')
    client = node.create_client(DockSw, 'dock_control')
    publisher = node.create_publisher(Bool,'/dock_end',10)

    eyantra_warehouse_dir = get_package_share_directory('eyantra_warehouse')

    config_path = os.path.join(eyantra_warehouse_dir, 'config', 'config.yaml')

    if not os.path.exists(config_path):
        print(f"ERROR: Config file not found at: {config_path}")

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    positions = config.get('position', {})
    package_ids = config.get('package_id', [])

    positions1 = positions.copy()

    id1 = str(package_ids[1])
    id2 = str(package_ids[2])

    id11 = package_ids[1]
    id12 = package_ids[2]

    pose1 = []

    for rack_pose in positions:
        for rack, pose in rack_pose.items():
            if rack.lower() == f"rack{id11}":
                pose1 = pose


    x1,y1,yaw1 = pose1[0], pose1[1], pose1[2]

    for rack_pose in positions:
        for rack, pose in rack_pose.items():
            if rack.lower() == f"rack{id12}":
                pose1 = pose
    
    x2,y2,yaw2 = pose1[0], pose1[1], pose1[2]


    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0    
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    def nav2goal(x1,y1,yaw1,x2,y2,yaw2,id,drop_goal,fr):

        [iz1,iw1] = euler_to_quaternion(yaw1)
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose1.pose.position.x = x1 #2.02+0.28
        goal_pose1.pose.position.y = y1 #-7.08   
        goal_pose1.pose.orientation.z = iz1 #-w+0.32 #-0.382#-w #- 0.32
        goal_pose1.pose.orientation.w = iw1 #-0.707 #z+0.1+0.12 #0.927#z #+0.1+0.1

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        navigator.goToPose(goal_pose1)

        i = 0
        
        while not navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500.0):
                    goal_pose1.pose.position.x = -3.0
                    navigator.goToPose(goal_pose1)

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
  

            request = DockSw.Request()
      
            request.orientation = yaw1
            request.docking = True 
            request.id = id
            request.drop_goal = drop_goal
            while not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Service not available, waiting again...') # Set this to True for docking
            print("id:", request.id)
            future = client.call_async(request)
            while rclpy.ok():
                rclpy.spin_once(node)
                if future.done():
                    res = future.result()
                    print(res)
                    break

        elif result == TaskResult.CANCELED:
            print('Goal was cancelled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        if fr == True:

            [iz3,iw3] = euler_to_quaternion(-3.14)
            goal_pose3 = PoseStamped()
            goal_pose3.header.frame_id = 'map'
            goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose3.pose.position.x = 1.6
            goal_pose3.pose.position.y = 0.0
            goal_pose3.pose.orientation.z = iz3
            goal_pose3.pose.orientation.w = iw3


            navigator.goToPose(goal_pose3)

            i = 0
            while not navigator.isTaskComplete():

                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500.0):
                        goal_pose1.pose.position.x = -3.0
                        navigator.goToPose(goal_pose1)


            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')

            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
            

        [iz2,iw2] = euler_to_quaternion(yaw2)
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose2.pose.position.x = x2
        goal_pose2.pose.position.y = y2
        goal_pose2.pose.orientation.z = iz2
        goal_pose2.pose.orientation.w = iw2


        navigator.goToPose(goal_pose2)

        i = 0
        while not navigator.isTaskComplete():

            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500.0):
                    goal_pose1.pose.position.x = -3.0
                    navigator.goToPose(goal_pose1)


        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            request.orientation = yaw2
            request.docking = False  # Set this to True for docking
            request.id = id
            request.drop_goal = drop_goal
            while not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Service not available, waiting again...')
            future = client.call_async(request)
            while rclpy.ok():
                print("running client node")
                rclpy.spin_once(node)
                if future.done():
                    res = future.result()
                    print(res)
                 
                    break
                
            msg = Bool()
            msg.data = True
            publisher.publish(msg)         


        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    msg = Bool()
    msg.data = True
    publisher.publish(msg)

    nav2goal(x1-1,y1+0.005-0.065+0.15-0.05 ,yaw1,0.11,-2.45+0.1,-3.14,id1,1.05-0.2,False)
    nav2goal(x2+0.20,y2-1,yaw2,1.61-0.4+0.34,-3.15-1,-1.57,id2,-3.15-0.15,True)  

    navigator.lifecycleShutdown()

    exit(0)
if __name__ == '__main__':
    main()
