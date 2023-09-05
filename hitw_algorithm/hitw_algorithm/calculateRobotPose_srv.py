import rclpy
import cv2

from rclpy.node import Node

from hitw_msgs.srv import CalculateRobotPose
from .submodules.image_processing import findRobotAngles

class robotPoseService(Node):
    def __init__(self):
        super().__init__('robotpose_service')

        self.calculateRobotPose_srv = self.create_service(
            CalculateRobotPose, 
            'calculate_robot_pose', 
            self.calculate_robot_pose_callback
        )

    def calculate_robot_pose_callback(self, request, response):
        img_original = cv2.imread('/home/razeragon/hole_in_the_wall/src/hole_in_the_wall/hitw_algorithm/images/Test_Hole.png')

        angles = findRobotAngles(img_original, request.link1_length, request.link2_length)
        response.joint_positions = angles

        print(request.link1_length, request.link2_length)

        self.get_logger().info('Incoming request\nLink 1 Length: {link1} Link 2 Length: {link2}'.format(link1=request.link1_length, link2=request.link2_length))

        return response
    
def main(args=None):
    rclpy.init(args=args)

    robotpose_service = robotPoseService()

    rclpy.spin(robotpose_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()