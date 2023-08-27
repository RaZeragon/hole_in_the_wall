import cv2

# Read original image
img = cv2.imread('testimage2.png')

# Display original image
cv2.imshow('Original', img)
cv2.waitKey(0)

# Convert to grayscale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Blur image
img_blur = cv2.GaussianBlur(img_gray, (3,3), 0)

# Canny Edge Detection
edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200)

# Display Canny Edge Detection image
cv2.imshow('Edges', edges)
cv2.waitKey(0)

cv2.destroyAllWindows()