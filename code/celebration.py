import time
from multiprocessing import Process, Queue, Value, Lock, Array
import sys
import subprocess
import pigpio
from motor_controller import motor_controller

#set fro broadcom numbering not board numbers
GPIO.setmode(GPIO.BCM)
# Set Up Pan Tilt Servo HardwarePWM Pins
motorPinT = 12
# connect to pi gpio Daemon
pi_hw = pigpio.pi()

currentDutyCycleT = 115000 # look at forward 

#Rotate the camera to make the balloon in the center of the image
def rotate_camera(dir, strength):
	global currentDutyCycleT
	full_up = 65000 # 1.3ms -> up
	full_down = 115000 # 2.3ms -> forward
	increment = (full_down - full_up) / 200  #200
	#camera up
	if dir == 1: 
		currentDutyCycleT = currentDutyCycleT - strength * increment
		if currentDutyCycleT < full_up:
			currentDutyCycleT = full_up
	
	elif dir == 0: 
		pi_hw.hardware_PWM(motorPinR, 50, 0) #50 Hz Freq. 0% duty cycle
		pi_hw.hardware_PWM(motorPinT, 50, 0) #50 Hz Freq. 0% duty cycle
		currentDutyCycleT = currentDutyCycleT - strength * increment
		if currentDutyCycleT < full_up:
			currentDutyCycleT = full_down
	#camera down
	else: 
		currentDutyCycleT = currentDutyCycleT + strength * increment 
		if currentDutyCycleT > full_down:
			currentDutyCycleT = full_down
	pi_hw.hardware_PWM(motorPinT, 50, currentDutyCycleT) #50 Hz Freq.
	##print("rotate_camera Function :  change the camera\n")
def wheels(): 
	
if __name__ == '__main__': 
	run_flag = Value('i',1)
	p0 = Process(target=tilt, args=())
	p1 = Process(target=wheels, args=())
	p0.start()
	p1.start()
	
	p0.join()
	p1.join()

	pi_hw.hardware_PWM(motorPinT, 0, 0) # 0hz, 0% duty cycle -- stop the motor!
	pi_hw.stop() # close pi gpio DMA resources
	
	
controller = motor_controller()

value = 1
start_time = time.time()
while 15>(time.time() - start_time):
    if value == 1:
        controller.set_control( 0., 30.)
        time.sleep(2)
        value += 1
    if value == 2:
        controller.set_control( 0., -30.)
        time.sleep(2)
        value += 1
    if value == 3:
        controller.set_control( 2000., 0.)
        time.sleep(1)
        value += 1
    if value == 4:
        controller.set_control( -2000., 0.)
        time.sleep(2)
        value = 1
