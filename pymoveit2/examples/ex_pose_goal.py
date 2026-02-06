#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import math

def euler_to_quaternion(roll, pitch, yaw):
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

    return [x,y,z,w]

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    # node.declare_parameter("position", [0.35, 0.10, 0.68])
    # node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
    # node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    # position = node.get_parameter("position").get_parameter_value().double_array_value
    # quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    # cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    # node.get_logger().info(
    #     f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    # )
    moveit2.move_to_pose([0.189, 0.108, 0.470],euler_to_quaternion(0,1.57,-1.57))
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()