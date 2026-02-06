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

    

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    

    # while not client.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('Service not available, waiting again...')

    config_path = os.path.join(os.path.pardir, '..', 'eyantra_warehouse', 'config', 'config.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    positions = config.get('position', {})
    package_ids = config.get('package_id', [])

    print("pos: ", positions)

    positions1 = positions.copy()

    # position_values = {}

    

    # for rack_info in positions:
    #     rack, coordinates = rack_info.popitem()
    #     position_values[rack] = coordinates

    # print("posval: ",position_values)


    # print("packa: ", position_values, package_ids)

    # pos=[]
    # # for j in range(3):
    # #     pos.append(position_values.items(j))

    # for h, p in position_values.items():
    #     pos.append(p)
    
    # print("pos: ",pos)

    # x2,y2,yaw2 = pos[2]

    id1 = str(package_ids[1])
    id2 = str(package_ids[2])

    id11 = package_ids[1]
    id12 = package_ids[2]

    print("id1: ",id11)

    pose1 = []

    print("pos1: ",positions1)

    # if id11 in package_ids:
    for rack_pose in positions:
        print("rckpose: ",rack_pose)
        for rack, pose in rack_pose.items():
            print("rack: ",rack.lower(),pose)
            if rack.lower() == f"rack{id11}":
                pose1 = pose

    print("pose1: ",pose1)

    x1,y1,yaw1 = pose1[0], pose1[1], pose1[2]

    for rack_pose in positions:
        print("rckpose: ",rack_pose)
        for rack, pose in rack_pose.items():
            print("rack: ",rack.lower(),pose)
            if rack.lower() == f"rack{id12}":
                pose1 = pose

    print("pose1: ",pose1)
    
    x2,y2,yaw2 = pose1[0], pose1[1], pose1[2]

    print("xyya324w: ",x2,y2,yaw2)

    # package_id = config.get('package_id', [])[1]
    # package_id1 = config.get('package_id', [])[2]

    
    # pi = config.get('package_id', [])
    # position_data = config.get('position', [])

    # if package_id in pi:
    #     for rack_pose in position_data:
    #         for rack, pose in rack_pose.items():
    #             if rack.lower() == f"rack{package_id}":
    #                 pose1 = pose
    
    # print("pose: ",pose1)
    # print("ID: ",pi[0])

    # pi1 = pi[0]

    # pi1 = config.get('package_id1', [])
    # position_data1 = config.get('position', [])

    # if package_id1 in pi1:
    #     for rack_pose in position_data1:
    #         for rack, pose in rack_pose.items():
    #             if rack.lower() == f"rack{package_id1}":
    #                 pose2 = pose
    
    # print("pose: ",pose1)
    # print("ID: ",pi[0])

    # pi1 = pi[0]

    # roll, pitch, yaw = 0, 0, pose1[2]
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # w = cy * cp * cr + sy * sp * sr
    # x = cy * cp * sr - sy * sp * cr
    # y = sy * cp * sr + cy * sp * cr
    # z = sy * cp * cr - cy * sp * sr

    # # return [w, x, y, z]
    # # x, y, z, w = 
    # print("quartenions func: ",z,w)



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

    # if pose1[2] == 1.57:
    #     x1 = pose1[0] + 0.15
    #     y1 = pose1[1] + 1.0
    
    # elif pose1[2] == -1.57:
    #     x1 = pose1[0] + 0.1
    #     y1 = pose1[1] - 1.0
    
    # elif pose1[2] == 3.14:
    #     x1 = pose1[0] - 1.0
    #     y1 = pose1[1] + 0.15
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
            # msg = Bool()
            # msg.data = True
            # publisher.publish(msg)
    

            request = DockSw.Request()
      
            request.orientation = yaw1
            request.docking = True 
            request.id = id
            request.drop_goal = drop_goal
            while not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Service not available, waiting again...') # Set this to True for docking
            # response = DockSw.Response()
            print("id:", request.id)
            future = client.call_async(request)
            # future = cli.call_async(req)
            while rclpy.ok():
                # print("jesus")
                rclpy.spin_once(node)
                if future.done():
                    res = future.result()
                    # node.get_logger().info("we made it with de bois")
                    print(res)
                    break
            print("we out the first one")
            # msg = Bool()
            # msg.data = False
            # publisher.publish(msg)
            # msg = Bool()
            # msg.data = True
            # publisher.publish(msg)
    #    print('Goal succeeded!')
            # msg = Bool()
            # msg.data = False
            # publisher.publish(msg)

            # node.destroy_node()
            # rclpy.shutdown()

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
            # goal_pose2 = PoseStamped()
            # goal_pose2.header.frame_id = 'map'
            # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
            # goal_pose2.pose.position.x = 0.05
            # goal_pose2.pose.position.y = -2.455
            # goal_pose2.pose.orientation.z = 0.707
            # goal_pose2.pose.orientation.w =  0.707
            # sanity check a valid path exists
            # path = navigator.getPath(initial_pose, goal_pose)

            navigator.goToPose(goal_pose3)

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
                # node2 = rclpy.create_node('dock_sw_client2')
                # # client2 = node2.create_client(DockSw, 'dock_control')


                # # while not client2.wait_for_service(timeout_sec=1.0):
                # #     node2.get_logger().info('Service not available, waiting again...')

                # print("\nDetaching!!")
                # # request = DockSw.Request()
                # # request.linear_dock = True
                # # request.orientation_dock = True
                # request.orientation = yaw2
                # request.docking = False  # Set this to True for docking
                # request.id = id
                # request.drop_goal = drop_goal
                # while not client.wait_for_service(timeout_sec=1.0):
                #     node.get_logger().info('Service not available, waiting again...')
                # print("id:", request.id)
                # future = client.call_async(request)
                # print("1")
                # # future = cli.call_async(req)
                # while rclpy.ok():
                #     print("running client node")

                #     rclpy.spin_once(node)
                #     if future.done():
                #         res = future.result()
                #         # node.get_logger().info("we made it with de bois")
                #         print(res)
                #         break


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
        # goal_pose2 = PoseStamped()
        # goal_pose2.header.frame_id = 'map'
        # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
        # goal_pose2.pose.position.x = 0.05
        # goal_pose2.pose.position.y = -2.455
        # goal_pose2.pose.orientation.z = 0.707
        # goal_pose2.pose.orientation.w =  0.707
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
            # msg = Bool()
            # msg.data = False
            # publisher.publish(msg)
            print('Goal succeeded!')
            # node2 = rclpy.create_node('dock_sw_client2')
            # client2 = node2.create_client(DockSw, 'dock_control')


            # while not client2.wait_for_service(timeout_sec=1.0):
            #     node2.get_logger().info('Service not available, waiting again...')

            print("\nDetaching!!")
            # request = DockSw.Request()
            # request.linear_dock = True
            # request.orientation_dock = True
            request.orientation = yaw2
            request.docking = False  # Set this to True for docking
            request.id = id
            request.drop_goal = drop_goal
            while not client.wait_for_service(timeout_sec=1.0):
                node.get_logger().info('Service not available, waiting again...')
            print("id:", request.id)
            future = client.call_async(request)
            print("1")
            # future = cli.call_async(req)
            while rclpy.ok():
                print("running client node")

                rclpy.spin_once(node)
                if future.done():
                    res = future.result()
                    # node.get_logger().info("we made it with de bois")
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

    # nav2goal(2.03-0.15,2.30,-1.57,-0.05,-2.45-1,-3.14,'2',1.05)
    # nav2goal(0.26,4.34+0.15,3.14,1.61,-4.15,-1.57,'1',-3.15)
    msg = Bool()
    msg.data = True
    publisher.publish(msg)

    # x1,y1,yaw1 = pos[1]
    # x2,y2,yaw2 = pos[2]

    # id1 = str(package_ids[1])
    # id2 = str(package_ids[2])

    # id11 = package_ids[1]
    # id12 = package_ids[2]

    # print("id1: ",id11)

    # pose1 = []

    # # if id11 in package_ids:
    # for rack_pose in positions:
    #     print("rckpose: ",rack_pose)
    #     for rack, pose in rack_pose.items():
    #         print("rack: ",rack.lower(),pose)
    #         if rack.lower() == f"rack{id11}":
    #             pose1 = pose

    # print("pose1: ",pose1)
    # x1,y1,yaw1 = pose1[0], pose1[1], pose1[2]

    # print("xyyaw: ",x1,y1,yaw1)

    nav2goal(x1-1,y1+0.005-0.065+0.15-0.05 ,yaw1,0.11,-2.45+0.1,-3.14,id1,1.05-0.2,False)
    nav2goal(x2+0.20,y2-1,yaw2,1.61-0.4+0.34,-3.15-1,-1.57,id2,-3.15-0.15,True)
   







    navigator.lifecycleShutdown()

    exit(0)
    # rclpy.shutdown()



if __name__ == '__main__':
    main()
