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

from rclpy.duration import Duration
from ebot_docking.srv import DockSw
from rclpy.executors import MultiThreadedExecutor


"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    config_path = os.path.join(os.path.pardir, '..', 'eyantra_warehouse', 'config', 'config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    package_id = config.get('package_id', [])[0]
    
    pi = config.get('package_id', [])
    position_data = config.get('position', [])

    if package_id in pi:
        for rack_pose in position_data:
            for rack, pose in rack_pose.items():
                if rack.lower() == f"rack{package_id}":
                    pose1 = pose
    
    print("pose: ",pose1)
    print("ID: ",pi[0])

    pi1 = pi[0]

    roll, pitch, yaw = 0, 0, pose1[2]
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    # return [w, x, y, z]
    # x, y, z, w = 
    print("quartenions func: ",z,w)



    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z =     
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = pose1[0] + 0.28
    goal_pose1.pose.position.y = pose1[1] + 1.09   
    goal_pose1.pose.orientation.w = -0.382 #-w+0.32 #-0.382#-w #- 0.32
    goal_pose1.pose.orientation.z = 0.927 #z+0.1+0.12 #0.927#z #+0.1+0.1

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose1)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

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

    # Do something depending on the return code
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        node = rclpy.create_node('dock_sw_client')
        client = node.create_client(DockSw, 'dock_control')


        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting again...')

        request = DockSw.Request()
        # request.linear_dock = True
        # request.orientation_dock = True
        request.orientation = yaw
        request.docking = True 
        request.id = str(pi1) # Set this to True for docking
        response = DockSw.Response()
        print("id:", request.id)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node,future)
        future.result()
        print(response)
    elif result == TaskResult.CANCELED:
        print('Goal was cancelled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')



    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = 'map'
    # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose2.pose.position.x = 0.0
    # goal_pose2.pose.position.y = -3.0
    # goal_pose2.pose.orientation.w = 0.4689747889950882
    # goal_pose2.pose.orientation.z =  0.8832115529574854

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.30
    goal_pose2.pose.position.y = -4.16
    goal_pose2.pose.orientation.z = -0.88
    goal_pose2.pose.orientation.w =  -0.44
   
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose2)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

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

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        node2 = rclpy.create_node('dock_sw_client2')
        client2 = node2.create_client(DockSw, 'dock_control')


        while not client2.wait_for_service(timeout_sec=1.0):
            node2.get_logger().info('Service not available, waiting again...')

        print("\nDetaching!!")
        request = DockSw.Request()
        # request.linear_dock = True
        # request.orientation_dock = True
        request.orientation = -1.57
        request.docking = False  # Set this to True for docking
        request.id = str(pi1)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node2,future)
        future.result()
        # client2.call_async(request)
       
        # rclpy.spin_until_future_complete(node,future)
        # future.result()
        # else:
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')






    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()