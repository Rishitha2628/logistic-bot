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
        self.subscriber_3 = self.create_subscription(Eefpos,'/eef_pos',self.callback_3,10)
        self.j = 0
        self.curr_pos = []
        # self.boxes = []
        # self.box_pos = []

    def callback_3(self,msg):
        # while self.j <=2 :
        # print("bruh")
        # self.curr_pos = [msg.x,msg.y,msg.Z]
        self.curr_pos = [msg.x,msg.y,msg.z]
        
        # self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        # self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.br = tf2_ros.TransformBroadcaster(self) 
        # self.timer2 = self.create_timer(1.0, self.on_timer(49))

    def current_position(self):
        rclpy.spin_once(self)
        # print(self.curr_pos)      
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

    
    def servo(self, end_point):
        # Define your movement sequence for NodeB here
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
            kp = 4.5
            kd = 0.3
            vel_x = kp * err_x + kd * (prev_x - err_x)
            vel_y = kp * err_y + kd * (prev_y - err_y)
            vel_z = kp * err_z + kd * (prev_z - err_z)
            print("\n",err_x)
            print("\n",err_y)
            print("\n",err_z)

            prev_x = err_x
            prev_y = err_y
            prev_z = err_z
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = vel_x
            twist_msg.twist.linear.y = vel_y
            twist_msg.twist.linear.z = vel_z
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0

            if np.abs(err_x)<=0.001 and np.abs(err_y)<=0.001:
                print("i did it fr")
                self.publisher.publish(twist_msg)
                break

            print("brev how laif")
            self.publisher.publish(twist_msg)
        # self.get_logger().info('Publishing TwistStamped2 message')



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
        # self.j = 0

        # self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        self.subscriber_1 = self.create_subscription(BoxOrientation,'/box_orientation',self.callback_1,10)
        # self.subscriber_2 = self.create_subscription(BoxTransformation,'/box_transformation',self.callback_2,10)
        self.j = 0
        self.boxes = []
        # self.box_pos = []




    def callback_1(self,msg):
    # while self.j <=2 :
    #     print("breh")
        self.boxes.append([msg.id,msg.position])
        # self.j = self.j + 1


    # def callback_2(self,msg):
    #     # while self.j <=2 :
    #         print("bruh")
    #         self.box_pos.append([msg.id,[msg.x,msg.y,msg.z]])
    #         # self.j = self.j + 1
        


class SubscriberNode2(Node):
    def __init__(self):
        super().__init__('sub2_node')
        # self.j = 0

        # self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
        # self.subscriber_1 = self.create_subscription(BoxOrientation,'/box_orientation',self.callback_1,10)
        self.subscriber_2 = self.create_subscription(BoxTransformation,'/box_transformation',self.callback_2,10)
        self.j = 0
        # self.boxes = []
        self.box_pos = []

    def callback_2(self,msg):
        # while self.j <=2 :
        #     print("bruh")
        self.box_pos.append([msg.id,[msg.x,msg.y,msg.z]])
            # self.j = self.j + 1
        


# class SubscriberNode3(Node):
#     def __init__(self):
#         super().__init__('sub3_node')
#         # self.j = 0

#         # self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        
#         # self.subscriber_1 = self.create_subscription(BoxOrientation,'/box_orientation',self.callback_1,10)
#         self.subscriber_3 = self.create_subscription(Eefpos,'/eef_pos',self.callback_3,10)
#         self.j = 0
#         self.curr_pos = []
#         # self.boxes = []
#         # self.box_pos = []

#     def callback_3(self,msg):
#         # while self.j <=2 :
#         print("bruh")
#         # self.curr_pos = [msg.x,msg.y,msg.Z]
#         self.curr_pos = [msg.x,msg.y,msg.z]
#         # self
#         # self.curr_pos.append([msg.x,msg.y,msg.z])
#         # rclpy.spin()


# def current_position():
#         rclpy.spin_once(SubscriberNode3())
#         curr_pos = SubscriberNode3().curr_pos
#         print("\n pos: ",curr_pos)
#         return curr_pos



def main(args=None):
    rclpy.init(args=args)

    joint_node = JointNode()
    servo_node = ServoNode()
    subscriber_node_1 = SubscriberNode()
    subscriber_node_2 = SubscriberNode2()
    # subscriber_node_3 = SubscriberNode3()

    
    i = 0


    while i <=2:
        rclpy.spin_once(subscriber_node_1)
        rclpy.spin_once(subscriber_node_2)
        # rclpy.spin_once(subscriber_node_3)

        i = i + 1

    # servo_node.current_position() 

    # rclpy.spin(subscriber_node_3)
        

    boxes_or = subscriber_node_1.boxes
    boxes_pos = subscriber_node_2.box_pos
    # curr_pos = subscriber_node_3.curr_pos
    
    
    print("\n",subscriber_node_1.boxes)
    print("\n",subscriber_node_2.box_pos)
    # print("\n",subscriber_node_3.curr_pos)



    # def current_position():
    #     rclpy.spin_once(subscriber_node_3)
    #     curr_pos = subscriber_node_3.curr_pos
    #     # print("\n pos: ",curr_pos)
    #     return curr_pos
    
    # print("\n pos: ",current_position())

    # print("\ncurr pos: ",curr_pos)

    

    # print("\n",servo_node.timer2)

    # self.timer2 = self.create_timer(1.0, self.on_timer)

    # servo_node.lookup(subscriber_node.boxes[0][0])
    # print("\n",subscriber_node.box_pos)



    combined_list = [{'id': item1[0], 'position': item1[1], 'coordinates': item2[1]} for item1 in boxes_or for item2 in boxes_pos if item1[0] == item2[0]]
    # print(combined_list)
    # print("\n",combined_list[0].get('position'))


    joint_positions_mid_point = [-4.0133371357775616e-05, -2.390015399808193, 2.4000294436934335, -3.1500128339504996, -1.5799936048616035, 3.1500000008613163] #mid position

    # joint_positions_2 = [-0.01873937989576291, -2.8562219797722173, 0.5651529865058404, 0.7205651286908856, -1.5714859559282885, -1.5900481023231006] #drop position (1)
    
    joint_positions_drop_point = [
    -0.022362056596619162,
    -1.8122501842494736,
    -1.3543250231207273,
    -3.117388739054292,
    -1.5927141228060662,
    -1.5722865453943542
]
    joint_positions_right = [0.457359703729443, -1.825791147751128, 2.1483332009431595, -3.467479178583467, -3.5986577591470676, -1.5745775643203812]

    joint_positions_left = [
    -2.5232231255688222,
    -3.1381719256474443,
    2.326311209323275,
    -2.332126712076181,
    2.5236545534638526,
    -3.1423085813887166
    ]

    joint_positions_mid_2 = [2.7682606908026095, -0.9177748962501258, -2.354704863129196, 0.13224989772419082, -4.340072729542147, -4.712442768370548]

    joint_positions_drop_2 = [
    0.455937876681463,
    -1.8144164470739272,
    -1.3536402923454012,
    -3.115456602503903,
    -1.1149312659492514,
    -1.5713006249006836
]
    right_box = [
    2.405810880972213,
    -2.9513732818554472,
    1.4279348621398982,
    -1.6174836340429701,
    0.7340980746928683,
    1.5703271415985474
]
    left_box = [
0.9504655684591299,
-1.2020922803021787,
1.0044004129347255,
-2.944767730168008,
-0.9516018443420373,
1.5717372229128688]

    mid_point = np.array([0.189, 0.108, 0.470])
    mid_point_2 = np.array([0.189, -0.150, 0.470])
    mid_right = np.array([0.189, 0.108, 0.470])
    mid_left = np.array([0.190, 0.109, 0.470])
    # box_point_1 = np.array([0.35, 0.1, 0.68])
    # box_point_2 = np.array([0.194, -0.43, 0.701])
    # drop_point = np.array([-0.60, 0.12, 0.397])

    # try:
    #     # Run the NodeA sequence
    #     joint_node.run_sequence()

    #     # Run the NodeB sequence
    #     servo_node.run_sequence()

    #     # rclpy.spin()
    #     # rclpy.spin(node_b)
    # except KeyboardInterrupt:
    #     pass

    # joint_node.destroy_node()
    # servo_node.destroy_node()
    # rclpy.shutdown()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(joint_node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()


  

    
    
    # for i in range(len(3)):
    def moving(dict):
        if dict.get('position') == 'center_left':
            joint_node.joint_position(joint_positions_mid_point)
            # servo_node.servo()
            # int_t_1 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            servo_node.servo(dict.get('coordinates'))
            time.sleep(1)
            servo_node.attach(dict.get('id'))
           
            # int_t_3 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.up(int_t_3)
            # time.sleep(1)
            
            # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            servo_node.servo(mid_point)
            time.sleep(1)
            
            joint_node.joint_position(joint_positions_drop_2)
            
            servo_node.detach(dict.get('id'))
            joint_node.joint_position(joint_positions_mid_point)

        elif dict.get('position') == 'center_right':
            joint_node.joint_position(joint_positions_mid_2)
            # servo_node.servo()
            # int_t_1 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            servo_node.servo(dict.get('coordinates'))
            time.sleep(1)
            servo_node.attach(dict.get('id'))
            # int_t_3 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.up(int_t_3)
            time.sleep(1)

            # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            servo_node.servo(mid_point_2)
            time.sleep(1)
            
            # joint_node.joint_position(joint_positions_drop_point)
            joint_node.joint_position(joint_positions_mid_point)

            joint_node.joint_position(joint_positions_drop_point)

            servo_node.detach(dict.get('id'))

            joint_node.joint_position(joint_positions_mid_point)


        elif dict.get('position') == 'left':
            # joint_node.joint_position(joint_positions_left)
            # # servo_node.servo()
            # int_t_1 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.servo(mid_left, dict.get('coordinates'), int_t_1,0.2/1.5)
            # time.sleep(1)
            # servo_node.attach(dict.get('id'))
            # int_t_3 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.up(int_t_3)
            # time.sleep(1)

            # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.servo(dict.get('coordinates'), mid_left, int_t_2,0)
            # time.sleep(1)
            
            # joint_node.joint_position(joint_positions_drop_point)
            # servo_node.detach(dict.get('id'))

            # joint_node.joint_position(joint_positions_mid_point)

            joint_node.joint_position(joint_positions_left)

            servo_node.servo(dict.get('coordinates'))
            time.sleep(1)
            servo_node.attach(dict.get('id'))
            servo_node.servo(mid_point)
            joint_node.joint_position(joint_positions_mid_point)


            # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.servo(dict.get('coordinates'), mid_right, int_t_2)
            # time.sleep(1)
            
            joint_node.joint_position(joint_positions_drop_point)
            servo_node.detach(dict.get('id'))

            joint_node.joint_position(joint_positions_mid_point)


        elif dict.get('position') == 'right':
            # joint_node.joint_position(joint_positions_right)
            # servo_node.servo()
            # int_t_1 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.servo(mid_right, dict.get('coordinates'), int_t_1)
            # time.sleep(1)
            joint_node.joint_position(right_box)

            servo_node.attach(dict.get('id'))

            joint_node.joint_position(joint_positions_mid_point)


            # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
            # servo_node.servo(dict.get('coordinates'), mid_right, int_t_2)
            # time.sleep(1)
            
            joint_node.joint_position(joint_positions_drop_point)
            servo_node.detach(dict.get('id'))

            joint_node.joint_position(joint_positions_mid_point)


    for i in range(3):
        moving(combined_list[i])




         
    # while i <=2:
    #     rclpy.spin_once(subscriber_node)
    #     i = i + 1

    # print(subscriber_node.boxes)


    # int_t_1 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
    # servo_node.servo(mid_point, box_point_1, int_t_1,0,1)
    # time.sleep(1)
    
    # servo_node.callback()
    # servo_node.subscriber()

    # print("\n bruh: ",servo_node.boxes)
    # time.sleep(1)
    # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
    # servo_node.servo(box_point_1, mid_point, int_t_2,0,1)
    # time.sleep(1)

    # joint_node.joint_position(joint_positions_mid_point)
    # joint_node.joint_position(joint_positions_drop_point)
    # servo_node.servo()
    # int_t_2 = float(servo_node.get_clock().now().to_msg().sec) + float(servo_node.get_clock().now().to_msg().nanosec) * (10 ** -9)
    # servo_node.servo(mid_point, box_point_1, int_t_2,0,1)
    # time.sleep(1)





if __name__ == '__main__':
    main()