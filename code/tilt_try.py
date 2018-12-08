import time
import RPi.GPIO as GPIO
import sys
import subprocess
import pigpio
#set fro broadcom numbering not board numbers
GPIO.setmode(GPIO.BCM)

# Set Up Pan Tilt Servo HardwarePWM Pins
motorPinT = 21
# connect to pi gpio Daemon
pi_hw = pigpio.pi()
pi_hw.hardware_PWM(motorPinT， 50， 75000） # 50hz, 7.5 % duty cycle(1.5 msec)
#pi_hw.hardware_PWM(motorPinT， 1000000， 500000） # 1Mhz, % duty cycle(1.5 msec)

time.sleep(20)

pi_hw.hardware_PWM(motorPinT，, 0, 0) # 0hz, 0% duty cycle -- stop the motor!

pi_hw.stop() # close pi gpio DMA resources

