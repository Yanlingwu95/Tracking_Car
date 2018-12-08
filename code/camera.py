# ECE 5725 OpenCV Example Code V1.0
# Xitang - 11/9/2018

# Code Description: This example code uses OpenCV 2.4.9 to stream video from Pi camera. 
# Pressing various keys on the keyboard would perform various actions. 

# Pressing "z" displays the shape, datatype of an image frame and the BGR value of center pixel at (319, 239)
# Pressing "x" sets the video resolution to be 320x240
# Pressing "c" sets the video resolution to be 640x480
# Pressing "v" turns on/off greyscale flag, which converts BGR image to greyscale image
# Pressing "b" turns on/off blur flag, which makes image blury
# Pressing "n" turns on/off sharpen flag, which sharpens image
# Pressing "m" turns on/off edge flag, which displays the edge of image
# Pressing "," turns on/off draw flag, which draws a circle at the center of image and some texts
# Pressing "q" exits the while loop and the program

# Import OpenCV Python library
import cv2
import numpy as np
print ("Running OpenCV version: " + str(cv2.__version__)) #2.4.9.1

# Create a VideoCapture class object to stream video from Pi camera
videoCap = cv2.VideoCapture(0)

# Use isOpened() to check if VideoCapture class object is created successfully
if (not videoCap.isOpened()):
	print ("Error: Can't find Pi Camera")
	quit()
else:
	print ("Success: Pi Camera is open")

# Use get() to obtain position 
videoWidth = videoCap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
videoHeight = videoCap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
print("Default video resolution: " + str(int(videoWidth))
	+ "x" + str(int(videoHeight)))

# Initialize some variables and kernel matrices
greyScaleFlag = blurFlag = sharpenFlag = edgeFlag = drawFlag = False
blurKernel = np.ones((3,3))/9.0
sharpenKernel = np.array([[0,-1,0], [-1,5,-1], [0,-1,0]])

while (True):
	returnBool, frame = videoCap.read()
	if (greyScaleFlag):
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	if (blurFlag):
		frame = cv2.filter2D(frame, -1, blurKernel)
	if (sharpenFlag):
		frame = cv2.filter2D(frame, -1, sharpenKernel)
	if (edgeFlag):
		frame = cv2.Canny(frame, 50, 200)
	if (drawFlag):
		cv2.circle(frame, (319,239), 30, (255,0,0))
		cv2.putText(frame, 'ECE 5725 - most fun class :D',
			(0,470), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1)

	cv2.imshow('ECE 5725 OpenCV Example', frame)
	keyPressed = cv2.waitKey(1)

	keyPressedChar = chr(keyPressed & 0xFF)
	# Perform various actions based on which key is pressed on keyboard
	if (keyPressedChar == 'z'):
		print ("-------------------------")		
		print ("Shape of frame: " + str(frame.shape) + " = (rows, columns, channels)")
		print ("Datatype of frame: " + str(frame.dtype) + ", meaning 8 bit unsigned int, value is between 0-255")
		print ("BGR Color Values of center pixel (319, 239) is: " + str(frame[239, 319]))
	elif (keyPressedChar == 'x'):
		videoCap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320)
		videoCap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
	elif (keyPressedChar == 'c'):
		videoCap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
		videoCap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
		#The camera is capable of 3280 x 2464 pixel static images
	elif (keyPressedChar == 'v'):
		greyScaleFlag = not greyScaleFlag
	elif (keyPressedChar == 'b'):
		blurFlag = not blurFlag
	elif (keyPressedChar == 'n'):
		sharpenFlag = not sharpenFlag
	elif (keyPressedChar == 'm'):
		edgeFlag = not edgeFlag
	elif (keyPressedChar == ','):
		drawFlag = not drawFlag
	elif (keyPressedChar == 'q'):
		break

videoCap.release()
cv2.destroyAllWindows() # Destroy the window created using imshow
