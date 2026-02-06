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
from std_msgs.msg import Float32MultiArray, Float32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import numpy as np
import time
from std_srvs.srv import Trigger


# from linkattacher_msgs.srv import AttachLink
# from linkattacher_msgs.srv import DetachLink
from usb_relay.srv import RelaySw



# Define a class for your ROS2 node

class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultrasonic_rl_callback, 10)
        # Add another one here
        # self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        self.imu_sub = self.create_subscription(Float32, 'orientation', self.imu_callback, 10)


        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
        # self.link_attach_cli = self.create_client(RelaySw, 'usb_relay_sw')
        # self.link_detach_cli = self.create_client(RelaySw, '/usb_relay_sw')

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


        # Initialize all  flags and parameters here
       
        #         
        # 
        # 
        # 
        # 
        # 

        # Initialize a timer for the main control loop
        # self.controller_timer = self.create_timer(0.1, self.controller_loop())

    # Callback function for odometry data
    def reset_odom(self):
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')

        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success== True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)

    def reset_imu(self):
        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)

    def switch_eletromagent(self,relayState):
        self.get_logger().info('Changing state of the relay to '+str(relayState))
        self.trigger_usb_relay = self.create_client(RelaySw, 'usb_relay_sw')
        while not self.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('USB Trigger Service not available, waiting...')

        request_relay = RelaySw.Request()
        request_relay.relaychannel = True
        request_relay.relaystate = relayState
        self.usb_relay_service_resp=self.trigger_usb_relay.call_async(request_relay)
        rclpy.spin_until_future_complete(self, self.usb_relay_service_resp)
        if(self.usb_relay_service_resp.result().success== True):
            self.get_logger().info(self.usb_relay_service_resp.result().message)
        else:
            self.get_logger().warn(self.usb_relay_service_resp.result().message)


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
        # self.usrleft_value = msg.range
        self.usrleft_value = msg.data[4]
        self.usrright_value = msg.data[5]

    # def ultrasonic_rr_callback(self, msg):
      
    #     self.usrright_value = msg.range

    def imu_callback(self, msg):
        
        # self.o_x = msg.orientation.x
        # self.o_y = msg.orientation.y
        # self.o_z = msg.orientation.z
        # self.o_w = msg.orientation.w

        # roll,pitch, yaw = euler_from_quaternion([self.o_x, self.o_y, self.o_z, self.o_w])
        # self.orientation = yaw
        self.orientation = msg.data   



    def convert_to_signed_angle(self,angle):
        return math.fmod(angle + math.pi, 2*math.pi) - math.pi


    # Main control loop for managing docking behavior

    def controller_loop(self,goal,bool):

                
        if not self.is_aligned:     #-->vector based 

            # if bool == False:
            vel = Twist()
            # error_z = goal - self.convert_to_signed_angle(self.orientation)
            vect_goal = np.array([math.cos(goal),math.sin(goal)])
            vect_imu = np.array([math.cos(self.orientation),math.sin(self.orientation)])
            error_z = math.acos(np.dot(vect_goal,vect_imu))
            kd = 0.2
            kp = 1.5 #1.0
            print("error_z: ",error_z)
            der = (error_z - self.prev_erz)
            # vel = Twist()
            # print("corrected angle: ",self.convert_to_signed_angle(self.orientation))
            vel.angular.z = kp*error_z #+ kd*der
            self.vel_pub.publish(vel)
            self.prev_erz = error_z
            # print("\nang vel: ",vel.angular.z)      
            print("\nangle error: ",error_z)      

            if np.abs(error_z)<0.04:
                # print("what")
                time.sleep(1)
                vel.angular.z = 0.0 
                self.is_aligned = True
                # print("\n breh2: ",self.is_aligned,self.is_docking)

                self.vel_pub.publish(vel)
                print("\nAttaching the rack!!")
                #Attaching
          


            print("\nAlignment done!!")

    
        elif self.is_aligned and self.is_docking:
            
            # error = 0
            # if self.usrleft_value>=0.2:
            time.sleep(1)
            self.switch_eletromagent(True)
            print("are we passed the future spin or nah?")
            # time.sleep(1)
            if bool == True:        #control system based of uls range which is in cm
                # print("hey?")
                vel = Twist()  
                error_x = self.usrleft_value - 13
                kp = 0.003 #0.88
                kd = 0.00 #0.08
                der = (error_x - self.prev_erx)
                vel.linear.x = - (kp * error_x + kd * der)
                # print("\n lin vel: ",vel.linear.x)
                # print("\nerror: ",error)
                self.vel_pub.publish(vel)
                # print("\n lin vel: ",vel.linear.x)
                self.prev_erx = error_x
                print("\nvel_x error: ",error_x)     
                print("\n uls value rn: ",self.usrleft_value) 


                if np.abs(error_x)<=2.0:    #change to centimetres if requried
                    # print(":(")
                    # vel = Twist()  
                    print("\nError threshold hit!")
                    vel.linear.x = 0.0   #
                    self.vel_pub.publish(vel)
                    intial_x = self.robot_pose[0]
                    final_x = self.robot_pose[0]

                    while  np.abs(intial_x-final_x)<=1.5:
                        vel.linear.x = 0.1
                        self.vel_pub.publish(vel)
                        final_x = self.robot_pose[0]

                    self.is_docking = False
                    print("\nDocking done!!")

            elif bool == False:         #control system based of odom pose which is in m

                vel = Twist()  
                # error_x = (3.15 + self.robot_pose[1]) #1.05
                error_x = 5.0 - np.abs(self.robot_pose[0])

                kp = 1.0 #0.78
                kd = 0.05 #0.05
                der = (error_x - self.prev_erx)

                vel.linear.x = - ( kp * error_x + kd * der)
                print("\n lin vel: ",vel.linear.x)
                self.vel_pub.publish(vel)

                self.prev_erx = error_x
                print("\nvel_x error: ",error_x)      


                if np.abs(error_x)<0.2:
                    print("\nError threshold hit!")

                    # print(":(")
                    # vel = Twist()  
                    vel.linear.x = 0.0   
                    self.vel_pub.publish(vel) 
                    self.is_docking = False
                    print("\nWent to the arm pose for picking!!")

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
 
        id = request.id
        docking = request.docking
        orientation = request.orientation
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        # if docking:
   
        if docking:
            while not self.is_aligned or self.is_docking:
            # self.get_logger().info("Waiting for alignment...")

                self.controller_loop(orientation,docking)
                # print("\n breh2: ",self.is_aligned,self.is_docking)
                rate.sleep()
            
            # print("yea?"

            # print("\nAttaching the rack!!")
            # #Attaching
            # time.sleep(1)
            # self.switch_eletromagent(True)
            # time.sleep(1)
            # future = self.link_attach_cli.call_async(req)
            # rclpy.spin_until_future_complete(self,future)
            # future.result()
            

        # Set the service response indicating success
            print("\nAttaching done :)")
            
            response.success = True
            response.message = "Docking control initiated"
            return response
        
        elif not docking:
            # self.is_docking = True
            # self.is_aligned = False 
            # print("broooo")

            while not self.is_aligned or self.is_docking:
            # self.get_logger().info("Waiting for alignment...")
            
                self.controller_loop(orientation,docking)
                # print("\n breh2: ",self.is_aligned,self.is_docking)
                rate.sleep()
            
            
            # print("yea?")


            #Detaching
            print("\nDetaching the rack!!")
            time.sleep(1)
            self.switch_eletromagent(False)
            time.sleep(1)

            # future = self.link_detach_cli.call_async(req)
            # rclpy.spin_until_future_complete(self,future)
            # future.result()
            # vel = Twist()

            # # j = 0
            # for j in range(3):
            #     vel.linear.x = 0.004
            #     self.vel_pub.publish(vel)

            # vel.linear.x = 0.0
            # self.vel_pub.publish(vel)

            


            

        # Set the service response indicating success
            print("\nDetaching done :)")
            
            response.success = True
            response.message = "Detach done initiated"
            return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    my_robot_docking_controller.reset_imu()
    my_robot_docking_controller.reset_odom()

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()