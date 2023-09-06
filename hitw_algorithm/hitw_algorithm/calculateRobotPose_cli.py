import sys
import rclpy
import cv2

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from hitw_msgs.srv import CalculateRobotPose

class robotPoseClient(Node):
    def __init__(self):
        super().__init__('robotpose_client')

        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            1
        )

        self.calculateRobotPose_cli = self.create_client(
            CalculateRobotPose, 
            'calculate_robot_pose'
        )

        while not self.calculateRobotPose_cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('CalculateRobotPose service not availble, waiting again...')
        self.calculateRobotPose_req = CalculateRobotPose.Request()

    def send_request(self):
        # self.calculateRobotPose_req.image = self.ros2_image
        self.calculateRobotPose_req.link1_length = 0.5
        self.calculateRobotPose_req.link2_length = 0.5

        self.future = self.calculateRobotPose_cli.call_async(self.calculateRobotPose_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    rclpy.init(args=args)

    robotpose_client = robotPoseClient()
    response = robotpose_client.send_request()
    print(response.joint_positions)
    robotpose_client.get_logger().info(
        'Result of CalculateRobotPose: Link 1 Angle: {angle1} Link 2 Angle: {angle2}'.format(angle1=response.joint_positions[0], angle2=response.joint_positions[1])
    )

    msg = Float64MultiArray()
    msg.data = response.joint_positions
    robotpose_client.joint_publisher.publish(msg)

    robotpose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()