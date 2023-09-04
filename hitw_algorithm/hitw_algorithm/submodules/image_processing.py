import cv2
import numpy as np
import math

# TO DO
# Update to C++

def imagePreProcess(image_path):
    # Preprocesses the image using following process
    img_original = cv2.imread('/home/razeragon/hole_in_the_wall/src/hole_in_the_wall/hitw_algorithm/images/Test_Hole.png')

    cv2.imshow('Original Image', img_original)
    cv2.waitKey(0)

    height, width, channels = img_original.shape

    # 1. Convert to Grayscale
    img_gray = cv2.cvtColor(img_original, cv2.COLOR_BGR2GRAY)

    # 2. Blur image
    img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)

    # 3. Canny edge detection
    img_edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

    # 4. Threshold image to standardize pixel values
    ret, img_thresh = cv2.threshold(img_edges, 127, 100, cv2.THRESH_BINARY)

    cv2.imshow('Threshold', img_thresh)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return height, width, img_original, img_gray, img_thresh

def buildOctants(circle_center_x, circle_center_y, x, y, octants):
    # Use the coordinates from first octant to fill in the rest of the circle
    # because of circular symmetry
    octants[0].append([circle_center_x + x, circle_center_y + y])
    octants[1].append([circle_center_x + y, circle_center_y + x])
    octants[2].append([circle_center_x + y, circle_center_y - x])
    octants[3].append([circle_center_x + x, circle_center_y - y])
    octants[4].append([circle_center_x - x, circle_center_y - y])
    octants[5].append([circle_center_x - y, circle_center_y - x])
    octants[6].append([circle_center_x - y, circle_center_y + x])
    octants[7].append([circle_center_x - x, circle_center_y + y])

def orderOctants(octants):
    # Orders the octants in a list of coordinates that form a continuous
    # loop to allow for searching along the circle
    circle_coordinates = []

    for index in range(1, len(octants), 2):
        quadrant = octants[index - 1] + octants[index][::-1]
        del quadrant[-1]
        circle_coordinates = circle_coordinates + quadrant

    return circle_coordinates

def circleBres(circle_center_x, circle_center_y, radius):
    # Generates coordinates for a circle of a specific center
    # and radius using the Bresenham's circle algorithm
    x = 0
    y = radius
    d = 3 - (2 * radius)
    octants = [[], [], [], [], [], [], [], []]
    buildOctants(circle_center_x, circle_center_y, x, y, octants)

    while (y >= x):
        x = x + 1

        if (d > 0):
            y = y - 1
            d = d + 4 * (x - y) + 10
        else:
            d = d + 4 * x + 6
        
        if (y >= x):
            buildOctants(circle_center_x, circle_center_y, x, y, octants)

    return(orderOctants(octants))

def createIntersectionPairs(intersection_list):
    # Creates a list of intersection pairs that describe how the circle interacts
    # with the hole outline
    intersection_pairs = []

    # Checks if there is either no intersections or one grouping
    if len(intersection_list) == 1:
        intersection_pairs = None
        return intersection_pairs
    
    # Takes the two closest points from the two groupings
    if len(intersection_list) == 2:
        intersection_pairs.append([[intersection_list[0][-2], intersection_list[1][0]], intersection_list[0][-1]])
        return intersection_pairs

    for index in range(len(intersection_list) - 1):
        intersection_pairs.append([[intersection_list[index][-2], intersection_list[index + 1][0]], intersection_list[index][-1]])

    intersection_pairs.append([[intersection_list[-1][-2], intersection_list[0][0]], intersection_list[-1][-1]])
    
    return intersection_pairs

def findIntersections(circle_coordinates, threshold_image, grayscale_image):
    # Iterates through the circle coordinates and tests if they're
    # within the image. If they are, then check if that pixel is
    # bright which means it is an edge of the hole and add that coordinate
    # to a grouping of intersections. A grouping of intersections is just
    # a chain of bright pixels 
    intersection_list = [[]]
    chain = False

    for coordinate in circle_coordinates:
        try:
            threshold_image[coordinate[1], coordinate[0]]
        except:
            # Probably add code to check for intersections with axes here
            pass
        else:
            if threshold_image[coordinate[1], coordinate[0]] > 50:
                intersection_list[-1].append(coordinate)
                chain = True
                continue

            if not(chain):
                continue

            # If the chain breaks, append a 1 or 0 to the grouping
            # 1 -> Leads out of the hole
            # 0 -> Leads inside the hole
            if grayscale_image[coordinate[1], coordinate[0]] > 127:
                intersection_list[-1].append(1)
            else:
                intersection_list[-1].append(0)

            intersection_list.append([])
            chain = False

    # If last grouping is empty, delete it
    if not(intersection_list[-1]):
        del intersection_list[-1]

    # print(intersection_list)

    intersection_pairs = createIntersectionPairs(intersection_list)

    return intersection_pairs

def createAngles(intersection_pairs, circle_center_x, circle_center_y):
    # Creates potential angles for the joint
    angles = []

    for intersection_pair in intersection_pairs:
        # Skip if the intersection pair spans a circle section outside of the hole
        if intersection_pair[-1] == 1:
            continue

        # print(intersection_pair)
        starting_coord = intersection_pair[0][0]
        ending_coord = intersection_pair[0][1]

        # thresh[starting_coord[1], starting_coord[0]] = 255
        # thresh[ending_coord[1], ending_coord[0]] = 255

        # cv2.imshow('Threshold_2', thresh)
        # cv2.waitKey(0)

        # Something weird going on here when dealing with intersections -- will deal with later
        starting_angle = math.atan2(circle_center_y - starting_coord[1], starting_coord[0] - circle_center_x)
        ending_angle = math.atan2(circle_center_y - ending_coord[1], ending_coord[0] - circle_center_x)

        angles.append((starting_angle + ending_angle) / 2)

    return angles

def calculateJointPosition(circle_center_x, circle_center_y, angle, link_length):
    # Calculates the position of a joint given an angle and a link length
    joint_center_x = int(circle_center_x + (math.cos(angle) * link_length))
    joint_center_y = int(circle_center_y - (math.sin(angle) * link_length))

    return [joint_center_x, joint_center_y]

def adjustJoints(joints):
    # Adjusts the first joint to the correct amount for ROS
    # Calculations -> +180 to -180 centered on +x axis
    # ROS          -> +180 to -180 centered on +y axis
    adjusted_joints = []

    # Calculate the first joint (base)
    if (joints[0] < 1.5708 and joints[0] >= -1.5708):
        parent_joint = (np.sign(joints[0]) * -1 * 1.5708) + joints[0]
        adjusted_joints.append(parent_joint)
    elif (joints[0] < -1.5708 and joints[0] >= 1.5708):
        parent_joint = joints[0] - (np.sign(joints[0] * 1.5708))
        adjusted_joints.append(parent_joint)

    # Calculates second joint if only two joints are present
    if len(joints) == 2:
        if ((joints[1] > -1.5708 and joints[1] < joints[0] - 1.5708) or
            (joints[1] < 1.5708 and joints[1] > 1.5708 + joints[0])):
            child_joint_angle = 3.1416 - joints[0] + joints[1]
            adjusted_joints.append(child_joint_angle)

            return adjusted_joints

        child_joint_angle = joints[1] - joints[0]
        adjusted_joints.append(child_joint_angle)

        return adjusted_joints


    # Calculate the positions of joints 2,3,4,...
    for index in range(1, len(joints) - 1):
        # Checks for a specific joint orientation
        if ((joints[index] > -1.5708 and joints[index] < joints[index - 1] - 1.5708) or
            (joints[index] < 1.5708 and joints[index] > 1.5708 + joints[index - 1])):
            child_joint_angle = 3.1416 - joints[index - 1] + joints[index]
            adjusted_joints.append(child_joint_angle)
            continue

        child_joint_angle = joints[index] - joints[index - 1]
        adjusted_joints.append(child_joint_angle)

    return adjusted_joints

def showRobotPose(original_image, joint_positions):
    # Display the positions of the joints and links over the original image
    joint_color = (0, 255, 0) # green
    link_color = (255, 0, 0) # blue

    # Color in the links
    for index in range(1, len(joint_positions)):
        new_image = cv2.line(original_image, (joint_positions[index - 1][0], joint_positions[index - 1][1]), (joint_positions[index][0], joint_positions[index][1]), link_color, 5)

    # Color in the joints
    for joint_position in joint_positions:
        new_image = cv2.circle(original_image, (joint_position[0], joint_position[1]), 5, joint_color, -1)

    return new_image

def findRobotAngles(image, link1_length_m, link2_length_m):

    height, width, img_original, img_gray, img_thresh = imagePreProcess(image)

    # Test ratio: 300 pixels | 1 m
    link1_length_pixels = int(link1_length_m * 300)
    link2_length_pixels = int(link2_length_m * 300)

    link1_circle_center_x = int(width / 2)
    link1_circle_center_y = height - 1

    link1_coordinates = circleBres(link1_circle_center_x, link1_circle_center_y, link1_length_pixels)
    link1_intersections = findIntersections(link1_coordinates, img_thresh, img_gray)
    link1_angles = createAngles(link1_intersections, link1_circle_center_x, link1_circle_center_y)
    link1_end_position = calculateJointPosition(link1_circle_center_x, link1_circle_center_y, link1_angles[0], link1_length_pixels)

    link2_coordinates = circleBres(link1_end_position[0], link1_end_position[1], link2_length_pixels)
    link2_intersections = findIntersections(link2_coordinates, img_thresh, img_gray)
    link2_angles = createAngles(link2_intersections, link1_end_position[0], link1_end_position[1])
    link2_end_position = calculateJointPosition(link1_end_position[0], link1_end_position[1], link2_angles[0], link2_length_pixels)

    joint_angles = [link1_angles[0], link2_angles[0]]
    adjusted_joints = adjustJoints(joint_angles)

    joint_positions = [[link1_circle_center_x, link1_circle_center_y], link1_end_position, link2_end_position]
    robot_image = showRobotPose(img_original, joint_positions)

    cv2.imshow('Robot Pose', robot_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return adjusted_joints