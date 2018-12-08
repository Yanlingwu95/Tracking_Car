from multiprocessing import Process, Queue, Value, Lock, Array
import time
import numpy as np
import cv2
from datetime import datetime


#Control the wheels
def control_wheels():
	print("control_wheels Funtion :  change the wheels\n")
	

#Rotate the camera to make the balloon in the center of the image
def rotate_camera(dir, strength):
	print("rotate_camera Function :  change the camera\n")
	


#Master Process --- recognize the red balloon
def recognize_balloon(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn, p_start_lock, p_end_lock): 
	last_contour_receive_time = 0
	start_time = 0
	start_queue = datetime.now()
	
	x_diff = 0
	y_diff = 0
	area = 0
	last_area = 0
	last_xdiff = 0
	last_ydiff = 0
	waiting_threshold = 10
	calibration = False
	
	while(run_flag.value): 
		#1. get a frame and show ret, 
		_, frame = cap.read() 
		# height 480; width 640; channel 3
		cv2.imshow('Capture', frame)
		#2. change to hsv model 
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
		#3. Threshold the HSV image to get only red colors and get mask 
		mask = cv2.inRange(hsv, lower_red, upper_red) #lower than 
		# print(mask)
		cv2.imshow('Mask_first', mask)
		start_time_count = time.time()
		
		# #find the contours 
		# contours,hierarchy = cv2.findContours(mask, 1, 2)
		
		cur_time = datetime.now()
		delta_time = cur_time - start_queue
		delta_time_tol_ms = delta_time.total_seconds()*1000
		
		if (calibration): 
			print("Calibration the camera")
			
		else: 
			#only put the mask in queue if it has past 10ms and there are less than 4 masks in queue
			if (delta_time_tol_ms > waiting_threshold) and (send_frame_queue.qsize() < 4) :
				start_queue = cur_time # Update last send to queue time
				send_frame_queue.put(mask)  # put mask in queue
				print("put the mask to queue\n")
				
			#check if receive_contour_queue is not qmpty
			if ((not receive_contour_queue.empty())):
				last_contour_receive_time = time.time()
				contours = receive_contour_queue.get() # Extract contour
				print("Main Processor: Get the processed contour from queue\n")
				
				Area_list = []
				if 0 == len(contours):
					time.sleep(waiting_threshold/1000.)
				else:
					Area_list = [ cv2.contourArea(c) for c in contours ]
					print (Area_list)
					maxindex = Area_list.index(max(Area_list))
					print maxindex
					cnt = contours[maxindex]#Get the first contour 
					#print (contours[0])
					M = cv2.moments(cnt)#Calculat M of the first contour
					# print (M)
					#calcualte the center coordinate
					if M['m00'] != 0:	
						cx = int(M['m10']/M['m00'])
						cy = int(M['m01']/M['m00'])
						area = M['m00']
						#PID control Algo to calculate strength to control servo
						x_diff = abs(cx - center_x)
						y_diff = abs(cy - center_y)
						print("x_bar=%f, y_bar= %f" % (cx,cy))
						print("x_diff= %f, y_diff= %f" % (x_diff,y_diff))
						kp_x_left = 3
						kd_x_left = 0.005
						
						kp_x_right = 3
						kd_x_right = 0.005
						
						kp_y = 3
						kd_y = 0.005
						
						proportional_x_left = x_diff/(x_res/2.0)
						proportional_x_right = x_diff/(x_res/2.0)
						proportional_y = y_diff/(y_res/2.0)
						
						derivative_x = (last_xdiff - x_diff)/(time.time() - start_time)
						derivative_y = (last_xdiff - y_diff)/(time.time() - start_time)
						derivative_z = (last_area - area)/(time.time() - start_time)
						
						start_time = time.time()
						#print("derivative_x: " + str(derivative_x))
						#print("derivative_x*kd_x: " + str(derivative_x*kd_x))
						start_time = time.time()
						strength_x_left = proportional_x_left*kp_x_left - derivative_x*kd_x_left
						strength_x_right = proportional_x_right*kp_x_right - derivative_x*kd_x_right
						strength_y = proportional_y*kp_y - derivative_y*kd_y
						#print "strength:"
						#print strength_x 
					
					#Assume the left and top corner is (0,0)
					if (x_diff <= x_tlr):
						a = 1
						#do nothing within tolerance range
					elif (cx > center_x):
						control_wheels()
						print('the car need to turn right and move forward\n')
					else:
						control_wheels()
						print('the car need to turn left and mvoe forward\n')
					if (y_diff <= y_tlr):
						# do nothing within tolerance range
						b = 1
					elif (cy < center_y):
						rotate_camera(1,strength_y)
						print('the camera need to raise its head up\n')
						
					else:
						rotate_camera(-1,strength_y)
						print('the camera need to low its head down\n')
					
					last_area = area
					last_xdiff = x_diff
					last_ydiff = y_diff
			
		if ((time.time()-last_contour_receive_time) < waiting_threshold/2000.):
			cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1) #Draw center of object
			cv2.drawContours(frame,contours,-1,(255,0,0),3) #Draw contour of object
		
		cv2.circle(frame, (center_x, center_y), 2, (0, 0, 255), -1) #Draw center of camera
		cv2.imshow('frame',frame) #Display Frame
		
		cal_time = time.time() - start_time_count
		print("Running_time = ", cal_time)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			run_flag.value = 0	
	
	print("Quit Processor 0")
	
def process_contour_1(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn, p_start_lock, p_end_lock):
	while (run_flag.value):
		startTime = datetime.now()
		startTime_ms = startTime.second *1000 + startTime.microsecond/1000
		# If frame queue not empty and it is Worker Process 1's turn
		if ((not send_frame_queue.empty()) and (p_start_turn.value == 1)):
			mask = send_frame_queue.get() # Grab a frame
			p_start_turn.value = 2 # and change it to worker process 2's turn
			print("Processor 1's Turn - Receive Mask Successfully")
			print(mask.shape)
			# # 1. Implement the open and close operation to get rid of noise and solidify an object
			maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
			maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
			# 2. Extract contour
			contours,h=cv2.findContours(maskClose.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			#find contours
			# contours,h=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			
			# Was going to implement a scheme that forces worker process to put frame
			# back in order, but didn't get to use
			#while (p_end_turn.value != 1):
			#	a = 1 # wait
			#p_end_turn.value = 2
			receive_contour_queue.put(contours) # Put contour back 
			print("processor_1 : put contour successfully\n")
		else:
			print("Processor 1 Didn't Receive Frame, sleep for 30ms")
			time.sleep(0.03)
		currentTime = datetime.now()
		currentTime_ms = currentTime.second *1000 + currentTime.microsecond/1000
		#print ("Processor 1 Processing Time: " + str(currentTime_ms-startTime_ms))
	print("Quiting Processor 1")

# Function for the Worker Process 2
def process_contour_2(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn, p_start_lock, p_end_lock):
	
	while (run_flag.value):
		startTime = datetime.now()
		startTime_ms = startTime.second *1000 + startTime.microsecond/1000
		if ((not send_frame_queue.empty()) and (p_start_turn.value == 2)):
			mask = send_frame_queue.get()
			p_start_turn.value = 3 # and change it to worker process 3's turn
			print("Processor 2's Turn - Receive Mask Successfully")
			print(mask.shape)
			# # 1. Implement the open and close operation to get rid of noise and solidify an object
			maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
			maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
			# 2. Extract contour
			contours,h=cv2.findContours(maskClose.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			#find the contours 
			# contours,h=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
		
			
			# Was going to implement a scheme that forces worker process to put frame
			# back in order, but didn't get to use
			#while (p_end_turn.value != 2):
			#	a = 1 # wait
			#p_end_turn.value = 3
			receive_contour_queue.put(contours)
			print("processor_2 : put contour successfully\n")
		else:
			print("Processor 2 Didn't Receive Frame, sleep for 30ms")
			time.sleep(0.03)
		currentTime = datetime.now()
		currentTime_ms = currentTime.second *1000 + currentTime.microsecond/1000
		#print ("Processor 2 Processing Time: " + str(currentTime_ms-startTime_ms))
	print("Quiting Processor 2")
	
# Function for the Worker Process 3
def process_contour_3(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn, p_start_lock, p_end_lock):
	
	while (run_flag.value):
		startTime = datetime.now()
		startTime_ms = startTime.second *1000 + startTime.microsecond/1000
		
		if ((not send_frame_queue.empty()) and (p_start_turn.value == 3)):
			mask = send_frame_queue.get()
			p_start_turn.value = 1 # and change it to worker process 1's turn
			print("Processor 3's Turn - Receive Mask Successfully")
			print(mask.shape)
				# # 1. Implement the open and close operation to get rid of noise and solidify an object
			maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
			maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
			#2. Extract contour
			contours,h=cv2.findContours(maskClose.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			#find the contours 
			# contours,h=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
			
			# Was going to implement a scheme that forces worker process to put frame
			# back in order, but didn't get to use
			#while (p_end_turn.value != 3):
			#	a = 1 # wait
			#p_end_turn.value = 1
			receive_contour_queue.put(contours)
			print("processor_3 : put contour successfully\n")
		else:
			print("Processor 3 Didn't Receive Frame, sleep for 30ms")
			time.sleep(0.03)
		currentTime = datetime.now()
		currentTime_ms = currentTime.second *1000 + currentTime.microsecond/1000
		#print ("Processor 3 Processing Time: " + str(currentTime_ms-startTime_ms))
	print("Quiting Processor 3")
	
	

# set red thresh 
lower_red = np.array([156,100,30])
#156, 100, 40

upper_red = np.array([180,255,255])

x_res = 640 #320 
y_res = 480 #240 
center_x = x_res/2
center_y = y_res/2
tolerance = 5 / 100.
x_tlr = x_res * tolerance
y_tlr = y_res * tolerance
# Setting Kernel Convolution Parameters
kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

cap = cv2.VideoCapture(0) 
# cap.set(3, x_res)
# cap.set(4, y_res)

if __name__ == '__main__':
	# run_flag is used to safely exit all processes
	run_flag = Value('i',1)
	# p_start_turn is used to keep worker processes process in order
	p_start_turn = Value('i', 1)
	p_end_turn = Value('i', 1)
	send_frame_queue = Queue()
	receive_contour_queue = Queue()
	p_start_lock = Lock() #Safety lock, but didnt use
	p_end_lock = Lock() #Safety lock, but didnt use
	
	p0 = Process(target=recognize_balloon, args=(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn, p_start_lock, p_end_lock))
	p1 = Process(target=process_contour_1, args=(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn,
		p_start_lock, p_end_lock))
	p2 = Process(target=process_contour_2, args=(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn,
		p_start_lock, p_end_lock))
	p3 = Process(target=process_contour_3, args=(run_flag, send_frame_queue, receive_contour_queue, p_start_turn, p_end_turn,
		p_start_lock, p_end_lock))
		
	p0.start()
	p1.start()
	p2.start()
	p3.start()
	# Wait for four processes to safely exit
	p0.join()
	p1.join()
	p2.join()
	p3.join()
	
	#Turn off cv2 window
	cap.release()
	cv2.destroyAllWindows() 		