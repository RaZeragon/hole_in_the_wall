import os
import sys
import math
import rclpy
import argparse
import numpy

from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage as tf
from functools import partial 

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            1
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            1
        )

    def target_pose_callback(self, msg):
        target_pose = msg.data

    def joint_states_callback(self, msg):
        joint_states = msg.position

        joint_states[0] = joint_states[0] + 1

        print(joint_states)

        #self.target_pose_publisher.publish(joint_states)

        self.adjust_joints(joint_states)

    def adjust_joints(self, joint_states):
        joint_states[0] = joint_states[0] + 1
        print(type(joint_states[0]))

        msg = JointState()
        msg.name = ["arm_joint_1", "arm_joint_2"]
        msg.position = joint_states
        #msg.velocity = []
        #msg.effort = []

        self.target_pose_publisher.publish(msg)

def main(args=None):

    # Initialize rlcpy library
    rclpy.init(args=args)

    # Create the node
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()