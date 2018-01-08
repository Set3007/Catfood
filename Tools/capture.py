#!/usr/bin/env python
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import io
from matplotlib import pyplot as plt
import argparse
import time
import RPi.GPIO as GPIO


# Pin in use
pin_plaque = [17,22,23,24]
pin_machoir = [19,16,26,20]
pinlight = 5
pinlight2 = 21
led = 4
capteur = 18
bp = 25
bpmanuel = 12

#vitesse
vitesse = 0.001 #temps de pause en ms
#attente
attentemona = 120

etape = []
num = 0
negnum = 0

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
#camera.framerate = 30
camera.vflip = True
camera.hflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)
  
print('Start!')

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

ap.add_argument("-i", "--info", 
	default="/home/pi/camera/photos/info.info",
	help="file info pixel count")

ap.add_argument("-c", "--cascade",
	default="/home/pi/cat-face-detector/result/BestMonaTop/cascade_134-0.xml",
	help="path to cat detector haar cascade")
args = vars(ap.parse_args())

detector = cv2.CascadeClassifier(args["cascade"])

fileinfo = open(args["info"], "w")

def captureneg(image):
	#capture neg
	global negnum
	negnum += 1
	imgnegpath = ("/home/pi/camera/neg/imageok%s.png"%negnum)
	cv2.imwrite(imgnegpath ,image)

	
def detect():
	global num
	loop = 0
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		captureneg(image)
		
		#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#rects = detector.detectMultiScale(gray, scaleFactor=1.25,
		#	minNeighbors=20, minSize=(75, 75))
		rects = "]"	
		
		if rects < "[" :		#Si detection
			print "capture!!!"
			for (i, (x, y, w, h)) in enumerate(rects):
				pw = (w * 15)/100
				ph = (h * 28)/100
				#img = frame[y+pw:(y-pw)+w,x+ph:(x-ph)+h]
				img = image[y:(y+h),x:(x+w)]
				imgsave = image
			nb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			#lower = np.array([6, 122, 70], dtype = "uint8")
			#upper = np.array([14, 220, 92], dtype = "uint8")
			#mask = cv2.inRange(hsv, lower, upper)
			#output = cv2.bitwise_and(img, img, mask = mask)
			ret,thresh = cv2.threshold(nb,127,255, cv2.THRESH_BINARY)
			blackp = cv2.countNonZero(thresh)
			print blackp
			
			#num += 1
			#imgpath = ("/home/pi/camera/photos/imageok%s.png"%num)
			#imgdetectpath = ("/home/pi/camera/photos/detectok%s.png"%num)
			#imgnb = ("/home/pi/camera/photos/nb%s.png"%num)
			#cv2.imwrite(imgpath ,imgsave)
			#cv2.imwrite(imgdetectpath ,img)
			#cv2.imwrite(imgnb ,nb)
			#infotxt =  '%s %s' % (num,blackp)
			#fileinfo.write('\n')
			#fileinfo.write(infotxt)
			#rawCapture.truncate(0)
		
		else:
			loop = loop + 1
			print "not detected"
			rawCapture.truncate(0)
		if loop == 250:
			return False
			break

def light():
	GPIO.output(pinlight,GPIO.HIGH)
	GPIO.output(pinlight2,GPIO.HIGH)

def lightoff():
	GPIO.output(pinlight,GPIO.LOW)
	GPIO.output(pinlight2,GPIO.LOW)

def ir():
	GPIO.output(led, GPIO.HIGH)

def setup():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(pinlight, GPIO.OUT, initial = GPIO.LOW)
	GPIO.setup(pinlight2, GPIO.OUT, initial = GPIO.LOW)
	GPIO.setup(capteur,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(led,GPIO.OUT)
	GPIO.setup(bp,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(bpmanuel,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	

if __name__ == '__main__':
	try:
		setup()
		ir()
		while True:
			time.sleep(0.01) #avoid inflating the CPU
			if GPIO.input(capteur) == 0:
				light()
				if detect():
					lightoff()
				lightoff()
			if GPIO.input(bpmanuel) == 0:
				openning()
				while GPIO.input(bpmanuel) == 1:
					time.sleep(0.01) #do something
				closing()

	except KeyboardInterrupt:
		GPIO.cleanup()
		fileinfo.close()

	
