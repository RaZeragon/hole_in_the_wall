import os
import sys
import math
import rclpy
import argparse
import numpy

from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState, Image
from tf2_msgs.msg import TFMessage as tf
from functools import partial 

from .submodules.image_processing import findRobotAngles

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            1
        )

        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            1
        )

    def camera_callback(self, msg):
        pass

def main(args=None):

    # Initialize rlcpy library
    rclpy.init(args=args)

    angles = findRobotAngles('testimage2.png', 0.5, 0.5)
    print(angles)

    # Create the node
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()