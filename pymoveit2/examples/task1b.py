#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from pymoveit2 import MoveIt2
import numpy as np
import time
from rclpy.callback_groups import ReentrantCallbackGroup


class PoseGoalExample(Node):
    def __init__(self):
        super().__init__("ex_pose_goal")
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.callback_group = ReentrantCallbackGroup()


        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]

    def move_to_pose(self, position, quat_xyzw, cartesian=False):
        self.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        print("bruh")
        # self.moveit2.wait_until_executed()

    def servo_to_pose(self, mid_point, end_point, int_t):
        # Define your movement sequence for NodeB here
        twist_msg = TwistStamped()
        # print(self.int_t)
        # int_t = self.get_clock().now().to_msg().sec
        
        # print(time.time() - self.int_t)# twist_msg.header.stamp = self.get_clock().now().to_msg()
        
        # self.mid_1 = np.array([0.189,0.108,0.470])
        # self.box_1 = np.array([0.35,0.1,0.68])
        vel = end_point - mid_point
        dist = np.linalg.norm(vel)
        vel = (1/3)*vel
        print(dist)
        # t1 = time.time() - self.int_t
        t1 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9) - int_t
        # print(t1)
        curr_vect = np.array([vel[0]*t1,vel[1]*t1,vel[2]*t1])
        dist_vect = np.linalg.norm(curr_vect)
        # print(dist_vect)
        # if (t1)<=9.0:
        while (dist_vect<=3*dist):
            t1 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9) - int_t
            curr_vect = np.array([vel[0]*t1,vel[1]*t1,vel[2]*t1])
            dist_vect = np.linalg.norm(curr_vect)
            print("\n total: ",dist)
            print("\n curr: ",dist_vect)
            print("\n time: ",t1)
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = vel[0]
            twist_msg.twist.linear.y = vel[1]
            twist_msg.twist.linear.z = vel[2]
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

    def run_sequence_1(self):
        mid_point = np.array([0.189, 0.2, 0.470])
        # mid_point_2 = np.array([])
        box_point_1 = np.array([0.35, 0.1, 0.68])
        box_point_2 = np.array([0.194, -0.43, 0.701])
        drop_point = np.array([-0.599, 0.120, 0.397])
        drop_point_2 = np.array([-0.60, -0.120, 0.397])

        # drop_point_2 = np.array([-0.600, 0.119, 0.396])

        # duration = 3.0

        # Servo to mid_point
        # self.int_t_1 = float(self.get_clock().now().to_msg().sec) + (float(self.get_clock().now().to_msg().nanosec) * (10 ** -9))
        # self.servo_to_pose(mid_point, box_point_1, self.int_t_1)
        # time.sleep(1)
        # self.int_t_2 = float(self.get_clock().now().to_msg().sec) + (float(self.get_clock().now().to_msg().nanosec) * (10 ** -9))
        # self.up(self.int_t_2)
        # time.sleep(1)


        # # self.move_to_pose([0.189, -0.150, 0.470], [0.501, -0.500, 0.500, 0.499])
        self.move_to_pose([0.2245914345954736, 0.5701980807920453, 0.6568660568905239], self.euler_to_quaternion(0,1.57,0))

        # # # time.sleep(1)

        # # self.move_to_pose(mid_point, self.euler_to_quaternion(0,1.57,0))
        # # # time.sleep(1)
        # # # time.sleep(0.5)

        # self.int_t_3 = float(self.get_clock().now().to_msg().sec) + (float(self.get_clock().now().to_msg().nanosec) * (10 ** -9))
        # self.servo_to_pose(box_point_1, mid_point, self.int_t_3)
        # time.sleep(1)

        # time.sleep(0.5)


        

        # Move to box_point
        # time.sleep(0.5)
        # self.move_to_pose(drop_point, self.euler_to_quaternion(0,0,0))
        # self.move_to_pose(drop_point, self.euler_to_quaternion(0,-1.57,1.57))

        # self.int_t_3 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9)
        # self.servo_to_pose(drop_point, mid_point, self.int_t_3,0.0,1)
        # self.move_to_pose(drop_point_2, self.euler_to_quaternion(0,-1.57,1.57))
        # self.move_to_pose(drop_point, self.euler_to_quaternion(0,-1.57,1.57))


        # self.moveit2.wait_until_executed()
        # time.sleep(0.5)

        # self.move_to_pose([0.222511327744926, -0.45610208473737945, 0.6568286270394272], self.euler_to_quaternion(0,-1.57,1.57))
        # # time.sleep(0.5)
        # self.move_to_pose(drop_point, self.euler_to_quaternion(0,0,0))


        # self.move_to_pose(mid_point, self.euler_to_quaternion(0,1.57,1.57))

        # self.move_to_pose(box_point_2, [0.505, 0.496, 0.499, 0.500])
        # self.move_to_pose(drop_point, [0.505, 0.496, 0.499, 0.500])


    # def run_sequence_2(self):
    #     mid_point = np.array([0.189, 0.108, 0.470])
    #     # mid_point_2 = np.array([])
    #     box_point_1 = np.array([0.35, 0.1, 0.68])
    #     box_point_2 = np.array([0.194, -0.43, 0.701])
    #     drop_point = np.array([-0.37, 0.12, 0.397])
    #     # duration = 3.0

    #     # Servo to mid_point
    #     self.int_t_2 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9)
    #     self.servo_to_pose(mid_point, box_point_2, self.int_t_2)

        # Move to box_point
        # time.sleep(0.5)
        # self.move_to_pose(drop_point, [0.505, 0.496, 0.499, 0.500])
        # # self.moveit2.wait_until_executed()
        # # time.sleep(0.5)

        # self.move_to_pose(mid_point, [0.505, 0.496, 0.499, 0.500])
        # self.move_to_pose(box_point_2, [0.505, 0.496, 0.499, 0.500])
        # self.move_to_pose(drop_point, [0.505, 0.496, 0.499, 0.500])


        # time.sleep(0.5)

        # self.int_t_2 = float(self.get_clock().now().to_msg().sec) + float(self.get_clock().now().to_msg().nanosec) * (10 ** -9)
        # self.servo_to_pose(mid_point, box_point_2, self.int_t_2)
        # # self.moveit2.wait_until_executed()
        # time.sleep(0.5)



        # Servo to mid_point again
        # self.servo_to_pose(box_point, mid_point, duration)

        # # Move to goal_point
        # self.move_to_pose(goal_point, [0.505, 0.496, 0.499, 0.500])

def main():
    rclpy.init()
    try:
        pose_goal_example = PoseGoalExample()
        pose_goal_example.run_sequence_1()
        # pose_goal_example.run_sequence_2()
        rclpy.spin(pose_goal_example)
    finally:
        pose_goal_example.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
