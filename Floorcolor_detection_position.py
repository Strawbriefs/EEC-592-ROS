
import cv2
import numpy as np
import rospy
from nav_msg.msg import Odometry

# Webcam parameters (your desired resolution)
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240

# Minimum required radius of enclosing circle of contour
MIN_RADIUS = 2

# Initialize camera and get actual resolution
cam = cv2.VideoCapture(0)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
camWidth = cam.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
camHeight = cam.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
print "Camera initialized: (" + str(camWidth) + ", " + str(camHeight) + ")"

# Main loop
while True:

    # Get image from camera
    ret_val, img = cam.read()

    # Blur image to remove noise
    img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)

    # Convert image from BGR to HSV
    img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)
	#get the turtlebot's current position(when the camera detect the color).
def callback(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y 
	rospy.loginfo('x: {}, y: {}'.format(x, y))
	#
def talker():
    rospy.Subscriber("/odom",Odometry, callback)
    rospy.init_node('color1')
	
	# loop over the red boundaries
boundaries1 = [([17, 15, 100], [50, 56, 200])]

	while (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		if __name__ == '__main__':
			try:
				talker()
		
#boundaries2 = [
#	([86, 31, 4], [220, 88, 50])
#	]
#	for (lower, upper) in boundaries:
	# create NumPy arrays from the boundaries
#		lower = np.array(lower, dtype = "uint8")
#		upper = np.array(upper, dtype = "uint8")
#		print(blue)
#boundaries3 = [	([25, 146, 190], [62, 174, 250])]
#	for (lower, upper) in boundaries:
#	# create NumPy arrays from the boundaries
#		lower = np.array(lower, dtype = "uint8")
#		upper = np.array(upper, dtype = "uint8")
#		print(yellow)

	# find the colors within the specified boundaries and apply
	# the mask
		mask = cv2.inRange(image, lower, upper)
		output = cv2.bitwise_and(image, image, mask = mask)

	# show the images
		cv2.imshow("images", np.hstack([image, output]))
		cv2.waitKey(0)

    # Dilate image to make white blobs larger
    img_binary = cv2.dilate(img_binary, None, iterations = 1)

    # Find center of object using contours instead of blob detection. From:
    img_contours = img_binary.copy()
    contours = cv2.findContours(img_contours, cv2.RETR_EXTERNAL, \
        cv2.CHAIN_APPROX_SIMPLE)[-2]

    # Find the largest contour and use it to compute the min enclosing circle
    center = None
    radius = 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius < MIN_RADIUS:
                center = None

     

    # Draw a green circle around the largest enclosed contour
    if center != None:
        cv2.circle(img, center, int(round(radius)), (0, 255, 0))

    # Show image windows
    cv2.imshow('webcam', img)
	cv2.imshow('binary', img_binary)
    cv2.imshow('contours', img_contours)
    cv2.waitKey(1) 
