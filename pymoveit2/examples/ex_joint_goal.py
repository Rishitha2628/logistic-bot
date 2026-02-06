#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57]"`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameter for joint positions
    # node.declare_parameter(
    #     "joint_positions",
    #     [
    #         -0.03403341798766224,
    #         -1.2848632387872256,
    #         -1.8567441129914095,
    #         -3.185621281551551,
    #         -1.545888364367352,
    #         3.1498768354918307
    #     ],
    # )

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

    # Get parameter
    # joint_positions = [
    #         -0.03403341798766224,
    #         -1.2848632387872256,
    #         -1.8567441129914095,
    #         -3.185621281551551,
    #         -1.545888364367352,
    #         3.1498768354918307
    # ]`#this was intial drop point for task1b`

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
    joint_positions_right = [
-1.5663887622158752,
-2.390097707845254,
2.4000034894366222,
-3.1499525280562466,
-1.5799850511105191,
3.150020103166123]

    joint_positions_left = [
1.5664887438429007,
-2.3899470290236065,
2.4001070854921163,
-3.1500176585659454,
-1.579891493302699,
3.1499292241519576

    ]


    joint_positions_mid_2 = [2.7682606908026095, -0.9177748962501258, -2.354704863129196, 0.13224989772419082, -4.340072729542147, -4.712442768370548]

#     joint_positions_drop_2 = [
#     -2.9171963008508346,
#     -1.241806902882956,
#     1.7466996260430494,
    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(joint_positions_mid_point)
    # moveit2.wait_until_executed()

#     -3.6456660787142066,
#     -1.344343760169889,
#     1.569272257458863
# ]

    drop_2 = [
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
    -5.567298461860854,
    -1.4971467057680017,
    1.3681747165486944,
    -3.0123904386668556,
    5.477944746211055,
    3.142381808305464

    ]
    left_2 = [
-2.5214143304399927,
-0.9967974383629605,
-2.3280602115933218,
0.184611583104628,
2.5223616544730927,
0.0012574791816151176
]
    idk = [0.0,0.0,0.0,0.0,0.0,1.57]

    new_mid_pos = [0.0, -2.398, 2.43, -3.15, -1.58, 3.15]
    new_drop_pos = [-0.027, -1.803, -1.3658, -3.039, -1.52, 3.15]
    # Move to joint configuration

    
    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(left_box)
    # moveit2.wait_until_executed()


    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(new_mid_pos)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(idk)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(new_drop_pos)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_drop_point)}}}")
    # moveit2.move_to_configuration(joint_positions_left)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(left_2)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(drop_2)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(joint_positions_mid_point)
    # moveit2.wait_until_executed()
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_2)}}}")
    moveit2.move_to_configuration(joint_positions_left)
    moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_drop_point)}}}")
    # moveit2.move_to_configuration(joint_positions_drop_2)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(joint_positions_mid_point)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_right)}}}")
    # moveit2.move_to_configuration(joint_positions_right)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_drop_point)}}}")
    # moveit2.move_to_configuration(joint_positions_drop_point)
    # moveit2.wait_until_executed()

    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_mid_point)}}}")
    # moveit2.move_to_configuration(joint_positions_mid_point)
    # moveit2.wait_until_executed()



    # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions_2)}}}")
    # moveit2.move_to_configuration(joint_positions_2)
    # moveit2.wait_until_executed()
    
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()