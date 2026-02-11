#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range 
from sensor_msgs.msg import Imu
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import numpy as np
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
import time


# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)


        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')

        # Create a publisher for sending velocity commands to the robot
        #
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_pose = [0,0,0]
        self.orientation = 0
        self.is_docking = True
        self.is_aligned = False 

        self.prev_erz = 0
        self.prev_erx = 0
        self.h = 0.01
        self.h2 = 0.1
        self.cond = False

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        roll,pitch, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range*100

    def ultrasonic_rr_callback(self, msg):
      
        self.usrright_value = msg.range*100

    def imu_callback(self, msg):
        
        self.o_x = msg.orientation.x
        self.o_y = msg.orientation.y
        self.o_z = msg.orientation.z
        self.o_w = msg.orientation.w

        roll,pitch, yaw = euler_from_quaternion([self.o_x, self.o_y, self.o_z, self.o_w])
        self.orientation = yaw


    def convert_to_signed_angle(self,angle):
       return math.fmod(angle + math.pi, 2*math.pi) - math.pi

    def controller_loop(self,goal,bool,g):   

        if not self.is_aligned:     
            vel = Twist()
            error_z = goal - self.convert_to_signed_angle(self.orientation)
            kd = 0.04/self.h
            kp = 1.8 #1.0

            der = (error_z - self.prev_erz)
            # vel = Twist()
            print("corrected angle: ",self.convert_to_signed_angle(self.orientation))
            vel.angular.z = kp*error_z #+ kd*der
            self.vel_pub.publish(vel)
            self.prev_erz = error_z
    
            print("\nangle error: ",error_z)      

            if np.abs(error_z)<0.04:
                vel.angular.z = 0.0 
                self.is_aligned = True
                self.vel_pub.publish(vel)

    
        elif self.is_aligned and self.is_docking:
           
            if bool == True:        #control system based of uls range which is in cm
                vel = Twist()  
                error_x = self.usrleft_value - 13
                kp = 0.02 #0.88
                kd = 0.001 #0.08
                der = (error_x - self.prev_erx)
                vel.linear.x = -( kp * error_x + kd * der )

                self.vel_pub.publish(vel)
                self.prev_erx = error_x
                print("\nvel_x error: ",error_x)      


                if np.abs(error_x)<=3:    #change to centimetres if requried
                    print("\nError threshold hit!")
                    vel.linear.x = 0.0   #
                    self.vel_pub.publish(vel) 
                    self.is_docking = False
                    print("\nDocking done!!")

            elif bool == False:
                self.get_logger().info("Detaching!!")
                vel = Twist()  
                if goal == 3.14 or goal == -3.14:
                    print("3.14")
                    error_x = g - np.abs(self.robot_pose[0])

                elif goal == 1.57 or goal == -1.57:
                    print("1.57")
                    error_x = g - self.robot_pose[1]


                kp = 0.78 #0.78
                kd = 0.05 #0.05
                der = (error_x - self.prev_erx)

                vel.linear.x = - ( kp * error_x + kd * der)
                print("\n lin vel: ",vel.linear.x)
                self.vel_pub.publish(vel)

                self.prev_erx = error_x
                print("vel_x error: ",error_x)      


                if np.abs(error_x)<0.12:
                    vel.linear.x = 0.0   
                    self.vel_pub.publish(vel) 
                    self.is_docking = False

            

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        self.is_docking = True
        self.is_aligned = False 
 
        id = request.id
        docking = request.docking
        orientation = request.orientation
        drop_goal = request.drop_goal
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        if docking:
            while not self.is_aligned or self.is_docking:
            # self.get_logger().info("Waiting for alignment...")

                self.controller_loop(orientation,docking,drop_goal)
                rate.sleep()
            

            while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Link attacher service not available, waiting again...')

            req = AttachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  'rack'+id   
            req.link2_name  = 'link'

            self.link_attach_cli.call_async(req)            

        # Set the service response indicating success
            response.success = True
            response.message = "Docking control initiated"
            print(response)
            
        elif not docking:

            while not self.is_aligned or self.is_docking:
            
                self.controller_loop(orientation,docking,drop_goal)
                rate.sleep()
            
            while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Link detacher service not available, waiting again...')

            req = DetachLink.Request()
            req.model1_name =  'ebot'     
            req.link1_name  = 'ebot_base_link'       
            req.model2_name =  'rack'+id   
            req.link2_name  = 'link'

        
            self.link_detach_cli.call_async(req)  
            response.success = True
            response.message = "Detach done initiated"
        self.cond = True
        return response
        

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()

    