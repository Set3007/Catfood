#!/usr/bin/env python
from picamera import PiCamera
from fractions import Fraction
from time import sleep
import RPi.GPIO as GPIO


# Pin in use
pinlight = 5
pinlight2 = 21

camera = PiCamera()
camera.resolution = (1280, 1024)
#camera.resolution = (640, 480)
#camera.resolution = (320, 240)
camera.vflip = True
camera.hflip = True
#camera.color_effects = (110, 110)
camera.contrast = 100
#camera.exposure_mode = 'spotlight'
#camera.meter_mode = 'matrix'
#camera.image_effect = 'gpen'
#camera.framerate=Fraction(1,1)
#camera.shutter_speed = 600000
#camera.iso = 800
#camera.exposure_mode = 'off'



def light():
	GPIO.output(pinlight,GPIO.HIGH)
	GPIO.output(pinlight2,GPIO.HIGH)

def lightoff():
	GPIO.output(pinlight,GPIO.LOW)
	GPIO.output(pinlight2,GPIO.LOW)
	
def setup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(pinlight, GPIO.OUT, initial = GPIO.LOW)
	GPIO.setup(pinlight2, GPIO.OUT, initial = GPIO.LOW)
		

if __name__ == '__main__':
	try:
		setup()
		while True:
			light()
			camera.start_preview()
			# Camera warm-up time
			sleep(60)

	except KeyboardInterrupt:
		GPIO.cleanup()
		camera.stop_preview()
		
GPIO.cleanup()
camera.stop_preview()
