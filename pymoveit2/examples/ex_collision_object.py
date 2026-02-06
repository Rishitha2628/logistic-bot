#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from pymoveit2 import MoveIt2
import numpy as np
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
import tf2_ros
from sensor_msgs.msg import CompressedImage, Image
from ur_description.msg import BoxOrientation
from ur_description.msg import BoxTransformation
from ur_description.msg import Eefpos
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
import os
from std_msgs.msg import Bool




class JointNode(Node):
    def __init__(self):
        super().__init__('joint_node')
        # self.publisher = self.create_publisher(String, 'message_topic', 10)
        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        

    def joint_position(self,joint_pos):
        print(f"Moving to {{joint_positions: {list(joint_pos)}}}")
        self.moveit2.move_to_configuration(joint_pos)
        self.moveit2.wait_until_executed()
        

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.j = 0

        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # self.publisher_2 = self.create_publisher(Bool, '/dock_end', 10)
        self.subscriber_3 = self.create_subscription(Eefpos,'/eef_pos',self.callback_3,10)
        self.j = 0
        self.curr_pos = []


    def callback_3(self,msg):
     
        self.curr_pos = [msg.x,msg.y,msg.z]
        


    def current_position(self):
        rclpy.spin_once(self)    
        return self.curr_pos
        

        
    def attach(self,id):
        gripper_control_attach = self.create_client(AttachLink, '/GripperMagnetON')

        while not gripper_control_attach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = AttachLink.Request()
        req.model1_name =  'box'+str(id)     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        gripper_control_attach.call_async(req)


    def detach(self,id):
        gripper_control_detach = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not gripper_control_detach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = DetachLink.Request()
        req.model1_name =  'box'+str(id)     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        gripper_control_detach.call_async(req)

    
    def servo(self, end_point,i):
   
        twist_msg = TwistStamped()
        prev_x = 0
        prev_y = 0
        prev_z = 0
        while True:
            # print("hello")
            mid_point = self.current_position()
            print("\n",mid_point)
            print("\n",end_point)
            err_x = end_point[0] - mid_point[0]
            err_y = end_point[1] - mid_point[1]
            err_z = end_point[2] - mid_point[2]
            # kp = 20
            kp = 20
            kd = 0.3
            vel_x = kp * err_x + kd * (prev_x - err_x)
            vel_y = kp * err_y + kd * (prev_y - err_y)
            vel_z = kp * err_z + kd * (prev_z - err_z)
            # print("\n",err_x)
            # print("\n",err_y)
            # print("\n",err_z)

            prev_x = err_x
            prev_y = err_y
            prev_z = err_z
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = vel_x
            twist_msg.twist.linear.y = vel_y
            twist_msg.twist.linear.z = vel_z
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            err_mag = np.sqrt(err_x**2 + err_y**2 + err_z**2)
            # print("\n err mag: ",err_mag)


            if i==0:

                if np.abs(err_mag)<=0.01:
                    print("i did it fr")
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.header.frame_id = 'base_link'
                    twist_msg.twist.linear.x = 0.0
                    twist_msg.twist.linear.y = 0.0
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = 0.0
                    self.publisher.publish(twist_msg)
                    # self.publisher.publish(twist_msg)
                    break

                print("brev how laif")
                self.publisher.publish(twist_msg)

            elif i==1:

                if np.abs(err_mag)<=0.2:
                    print("i did it fr")
                    twist_msg.header.stamp = self.get_clock().now().to_msg()
                    twist_msg.header.frame_id = 'base_link'
                    twist_msg.twist.linear.x = 0.0
                    twist_msg.twist.linear.y = 0.0
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = 0.0
                    self.publisher.publish(twist_msg)
                    # self.publisher.publish(twist_msg)
                    break

                print("brev how laif")
                self.publisher.publish(twist_msg)


    def up(self,int_t):
        twist_msg = TwistStamped()

        # vel = 0.2/3
        t1 = 0  
        
        while (t1<=3):
            print(t1)
            t1 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9) - int_t
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.067
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info('Publishing TwistStamped message')
        
        
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Publishing TwistStamped2 message')




class SubscriberNode(Node):
    def __init__(self):

        super().__init__('sub1_node')    
        self.subscriber_1 = self.create_subscription(BoxOrientation,'/box_orientation',self.callback_1,10)
        self.j = 0
        self.boxes = []


    def callback_1(self,msg):
        # self.boxes.pop(0)
        # self.boxes.append([msg.id,msg.position])
        self.boxes = [msg.id,msg.position]
  


class SubscriberNode2(Node):
    def __init__(self):

        super().__init__('sub2_node')
        self.subscriber_2 = self.create_subscription(BoxTransformation,'/box_transformation',self.callback_2,10)
        self.j = 0
        self.box_pos = [0]

    def callback_2(self,msg):
        # self.box_pos.pop(0)
        # self.box_pos.append([msg.id,[msg.x,msg.y,msg.z]])
        self.box_pos = [msg.id,[msg.x,msg.y,msg.z]]


class SubscriberNode3(Node):
    def __init__(self):

        super().__init__('sub3_node')
        self.subscriber_3 = self.create_subscription(Bool,'/arm_start',self.callback_3,10)
        # self.j = 0
        # self.box_pos = []
        self.condition = False

    def callback_3(self,msg):
        # self.box_pos.pop(0)
        # self.box_pos.append([msg.id,[msg.x,msg.y,msg.z]])
        # self.box_pos = [msg.id,[msg.x,msg.y,msg.z]]'
        self.condition = msg.data




def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('arm_bs')
    # publisher = node.create_publisher(Bool,'/arm_cond',10)
    publisher = node.create_publisher(Bool,'/dock_end',10)


    

    os.system("ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}")
    # boxes_or = []
    # boxes_pos = []   
    # combined_list = []

    

    
    i = 0

    joint_positions_mid_point = [0.0, -2.398, 2.43, -3.15, -1.58, 3.15]


    joint_positions_drop_point =[-0.027, -1.803, -1.3658, -3.039, -1.52, 3.15]
    
    joint_positions_right = [
-1.57,
-2.390097707845254,
2.4000034894366222,
-3.1499525280562466,
-1.5799850511105191,
3.1415]

    joint_positions_left = [
1.57,
-2.3899470290236065,
2.4001070854921163,
-3.1500176585659454,
-1.579891493302699,
3.1415]


    joint_positions_drop_2 =[0.3489, -1.803, -1.3658, -3.039, -1.52, 3.15]

    mid_point = np.array([0.189, 0.108, 0.470])
    mid_point_2 = np.array([0.189, -0.150, 0.470])
    mid_right = np.array([0.189, 0.108, 0.470])
    mid_left = np.array([0.190, 0.109, 0.470])
    mid_point_l = np.array([-0.108, 0.189, 0.469])
    mid_point_r = np.array([0.108, -0.189, 0.470])
    executor = rclpy.executors.MultiThreadedExecutor(2)
    joint_node = JointNode()
    servo_node = ServoNode()
    subscriber_node_1 = SubscriberNode()
    subscriber_node_2 = SubscriberNode2()
    subscriber_node_3 = SubscriberNode3()
    executor.add_node(joint_node)
    # executor.add_node(servo_node)
    # executor.add_node(subscriber_node_1)
    # executor.add_node(subscriber_node_2)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    while True:
        # try:
        print("hello")
        print(i)



    # while i <=2:
       
        rclpy.spin_once(subscriber_node_1)
        rclpy.spin_once(subscriber_node_2)
        rclpy.spin_once(subscriber_node_3)
        

            # i = i + 1

    

        boxes_or = subscriber_node_1.boxes
        boxes_pos = subscriber_node_2.box_pos
        condition = subscriber_node_3.condition
    
        
        
        print("\n",boxes_or)
        print("\n",boxes_pos)
        # print("\n",condition)




        # combined_list = [{'id': item1[0], 'position': item1[1], 'coordinates': item2[1]} for item1 in boxes_or for item2 in boxes_pos if item1[0] == item2[0]]
        # combined_list = [{'id': boxes_or[0], 'position': boxes_or[1], 'coordinates': boxes_pos[1]}]
        combined_list = [boxes_or[0],boxes_or[1],boxes_pos[1]]

        print("\n combined list: ",combined_list)
        # box_list = [boxes_or[0][0],boxes_or[0][1],boxes_pos[0][1]]




        def moving(dict):
            # msg = Bool()
            # msg.data = True
            # publisher.publish(msg)
            

            if dict[1] == 'center':
           
                joint_node.joint_position(joint_positions_mid_point)
                servo_node.servo(dict[2],0)
                servo_node.attach(dict[0])
                servo_node.servo(mid_point,1)
                joint_node.joint_position(joint_positions_drop_2)
                servo_node.detach(dict[0])
                msg = Bool()
                msg.data = False
                publisher.publish(msg)
                joint_node.joint_position(joint_positions_mid_point)   
                joint_node.joint_position(joint_positions_left)


               



            elif dict[1] == 'left':
    
                # servo_node.servo(dict.get('coordinates'),0)
                servo_node.servo(dict[2],0)
                servo_node.attach(dict[0])
                servo_node.servo(mid_point_l,1)
                time.sleep(1)
                joint_node.joint_position(joint_positions_drop_point)
                servo_node.detach(dict[0])
                msg = Bool()
                msg.data = False
                publisher.publish(msg)
                joint_node.joint_position(joint_positions_mid_point)



            elif dict[1] == 'right':
    
                # servo_node.servo(dict.get('coordinates'),0)
                joint_node.joint_position(joint_positions_right)
                servo_node.servo(dict[2],0)
                servo_node.attach(dict[0])
                servo_node.servo(mid_point_r,1)
                time.sleep(1)
                joint_node.joint_position(joint_positions_drop_point)
                servo_node.detach(dict[0])
                msg = Bool()
                msg.data = False
                publisher.publish(msg)
                joint_node.joint_position(joint_positions_mid_point)


            
 

        # for i in range(3):
        if condition == True:
            moving(combined_list)

        elif condition == False:
            continue
        
       
        i = i+1

        # except:
        #     print("rishi abuses me")






if __name__ == '__main__':
    main()