import cv2
import numpy as np
import math

# TO DO
# Test other versions of hole algorithm
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

# Create image for first link mask
link1_img = np.zeros((height, width), np.uint8)

# Link length (m)
# Test ratio: 300 pixels | 1 m
link1_length_m = 0.5
link1_length_pixels = int(link1_length_m * 300)

# Draw a circle over the link1_img with the radius = link length to simulate the joint path
circle_center_x = int(width / 2)
circle_center_y = height - 1
circle_width = 2
link1_img_circle = cv2.circle(link1_img, (circle_center_x, circle_center_y), link1_length_pixels, 100, circle_width)

cv2.imshow('Circle', link1_img_circle)
cv2.waitKey(0)

# Adds the two images together to see where the joint intersects the hole outline via increased pixel value
combined_images = thresh + link1_img_circle

cv2.imshow('Combined', combined_images)
cv2.waitKey(0)

# Create bounds for rectangle around the circle to iterate through the combined images
offset = circle_width - 1
lower_x = circle_center_x - link1_length_pixels - offset
upper_x = circle_center_x + link1_length_pixels + offset
lower_y = circle_center_y - link1_length_pixels - offset
upper_y = circle_center_y

intersection_list = []

# Iterates through the rectangle to check for intersections and append them to a list
for x_cord in range(lower_x, upper_x):
    for y_cord in reversed(range(lower_y, upper_y)):

        # combined_images[y_cord, x_cord] = combined_images[y_cord, x_cord] + 50
        if (combined_images[y_cord, x_cord] < 150):
            continue

        intersection_list.append([x_cord, y_cord])

# Next part assumes there were only two intersections
# Need to tell which pixels are grouped together and choose the edge pixels of those groupings
# Need to add error checking for edge cases
previous_x_coord = intersection_list[0][0]
starting_coordinate = []
ending_coordinate = []

for coordinate_id in range(1, len(intersection_list)):
    if (intersection_list[coordinate_id][0] > (previous_x_coord + 1)):
        ending_coordinate = intersection_list[coordinate_id]
        starting_coordinate = intersection_list[coordinate_id - 1]
        break

    previous_x_coord = intersection_list[coordinate_id][0]

starting_angle = math.atan2(starting_coordinate[1] - circle_center_y, circle_center_x - starting_coordinate[0])
ending_angle = math.atan2(ending_coordinate[1] - circle_center_y, circle_center_x - ending_coordinate[0])

first_angle = (starting_angle + ending_angle) / 2

cv2.imshow('Combined with square', combined_images)
cv2.waitKey(0)

cv2.destroyAllWindows()