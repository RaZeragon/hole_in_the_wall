import cv2
import numpy as np
import math

# TO DO
# Update to C++

# Read original image
img = cv2.imread('testimage2.png')

# Read image parameters
height, width, channels = img.shape

# Convert to grayscale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Blur image
img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)

# Canny Edge Detection
edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

# Threshold the image to standardize pixel values
ret, thresh = cv2.threshold(edges, 127, 100, cv2.THRESH_BINARY)

cv2.imshow('Threshold', thresh)
cv2.waitKey(0)

# Link length (m)
# Test ratio: 300 pixels | 1 m
link1_length_m = 0.5
link2_length_m = 0.5
link1_length_pixels = int(link1_length_m * 300)
link2_length_pixels = int(link2_length_m * 300)

circle_center_x = int(width / 2)
circle_center_y = height - 1

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

def findIntersections(circle_coordinates, threshold_image):
    # Iterates through the circle coordinates and tests if they're
    # within the image. If they are, then check if that pixel is
    # bright which means it is an edge of the hole and add that coordinate
    # to a list of intersections
    intersection_list = []

    for coordinate in circle_coordinates:
        try:
            threshold_image[coordinate[1], coordinate[0]]
        except:
            pass
        else:
            if threshold_image[coordinate[1], coordinate[0]] > 50:
                intersection_list.append(coordinate)

    return intersection_list

def createAnglePairs(intersection_list):
    pass

test_coordinates = circleBres(circle_center_x, circle_center_y, link1_length_pixels)
print(findIntersections(test_coordinates, thresh))

# # Next part assumes there were only two intersections
# # Need to tell which pixels are grouped together and choose the edge pixels of those groupings
# # Need to add error checking for edge cases
# previous_x_coord = intersection_list[0][0]
# starting_coordinate = []
# ending_coordinate = []

# for coordinate_id in range(1, len(intersection_list)):
#     if (intersection_list[coordinate_id][0] > (previous_x_coord + 1)):
#         ending_coordinate = intersection_list[coordinate_id]
#         starting_coordinate = intersection_list[coordinate_id - 1]
#         break

#     previous_x_coord = intersection_list[coordinate_id][0]

# # Could change to just averaging the two coordinate points
# # Could also do similar triangles?
# starting_angle = math.atan2(starting_coordinate[1] - circle_center_y, circle_center_x - starting_coordinate[0]) * (-1)
# ending_angle = math.atan2(ending_coordinate[1] - circle_center_y, circle_center_x - ending_coordinate[0]) * (-1)

# first_angle_rad = (starting_angle + ending_angle) / 2

# # Calculate position of first joint
# joint_center_x = circle_center_x - (math.cos(first_angle_rad) * link1_length_pixels)
# joint_center_y = circle_center_y - (math.sin(first_angle_rad) * link1_length_pixels)

# # print(joint_center_x, joint_center_y)
# # combined_images[int(joint_center_y), int(joint_center_x)] = 255


cv2.imshow('Second combined square', combined_images)
cv2.waitKey(0)

cv2.destroyAllWindows()