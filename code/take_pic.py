from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview(alpha=200)
for i in range(5):
	sleep(3)
	camera.capture('/home/pi/Pi_Luck/camera/pic/image%d.jpg' %i)
camera.stop_preview()