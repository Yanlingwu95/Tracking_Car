## Recognize the objects with specific color

import cv2 
import time
import numpy as np
import RPi.GPIO as GPIO
import sys
import subprocess
import pigpio
 
#set fro broadcom numbering not board numbers
GPIO.setmode(GPIO.BCM)
# Set Up Pan Tilt Servo HardwarePWM Pins
motorPinT = 12
# connect to pi gpio Daemon
pi_hw = pigpio.pi()

cap = cv2.VideoCapture(0) 
# set red thresh 
#lower_red=np.array([22, 31, 0]) #[78,43,46]
#upper_red=np.array([0, 100, 100]) 

# This one belongs to red:
lower_red = np.array([156,100,30])
upper_red = np.array([180,255,255])

x_res = 640 #320 
y_res = 480 #240 
center_x = x_res/2
center_y = y_res/2
x_diff = 0
y_diff = 0
tolerance = 5 / 100.
x_tlr = x_res * tolerance
y_tlr = y_res * tolerance
#time.sleep(1)
def rotate_camera(strength):
	dc = strength / 20 * 100 * 10000 #(1.5-2.3) # 1.5-> up  #2.3-> forward
	pi_hw.hardware_PWM(motorPinT, 50, dc) # 50hz, 7.5 % duty cycle(1.5 msec)	
	print("rotate_camera Function :  change the camera\n")
	

while(1): 
	# get a frame and show ret, 
	_, frame = cap.read() 
	# height 480; width 640; channel 3
	cv2.imshow('Capture', frame)
	# change to hsv model 
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
	# get mask 
	mask = cv2.inRange(hsv, lower_red, upper_red) #lower than 
	# print(mask)
	cv2.imshow('Mask', mask) 
	
	start_time = time.time()
	#contours, hierarchy = cv2.findContours(mask,mode = CV_RETR_TREE, method=CV_CHAIN_APPROX_SIMPLE)
	contours,hierarchy = cv2.findContours(mask, 1, 2)
	#print(contours)
	
	area = []
	if 0 == len(contours):
		time.sleep(0.02)
	else:
		#for c in contours:
		#	area.append(cv2.contourArea(c) )
		area = [ cv2.contourArea(c) for c in contours ]
		
		#print area
		maxindex = area.index(max(area))
		#print maxindex
		cnt = contours[maxindex]#Get the first contour 
		#print (contours[0])
		M = cv2.moments(cnt)#Calculat M of the first contour
		# print (M)
		#calcualte the center coordinate
		if M['m00'] != 0:	
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			x_diff = abs(cx - center_x)
			y_diff = abs(cy - center_y)
			print("x_bar=%f, y_bar= %f" % (cx,cy))
			print("x_diff= %f, y_diff= %f" % (x_diff,y_diff))
			print("area = %f" % area)
			if (y_diff <= y_tlr):
				# do nothing within tolerance range
				b = 1
			elif (cy < center_y):
				rotate_camera(1.5)
				print('the camera need to raise its head up\n')
				
			else:
				rotate_camera(2.3)
				print('the camera need to low its head down\n')
			
		#area = cv2.contourArea(cnt)
		print("Area = %f" %M['m00'])
	cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1) #Draw center of object
	cv2.drawContours(frame,contours,-1,(255,0,0),3) #Draw contour of object
	cv2.circle(frame, (center_x, center_y), 2, (0, 0, 255), -1) #Draw center of camera
	cv2.imshow('frame',frame) #Display Frame
	
	
	cal_time = time.time() - start_time
	#print("Start_time = ", cal_time)
	#text = "(x_bar,y_bar)"
	#font = cv2.FONT_HERSHEY_SUPLEX 
	# font=cv2.InitFont(cv.CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 1, 1, 0, 3, 8)
	#cv2.putText(mask,text, (300,300),font,2,(0,0,255), 1)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
