#!/usr/bin/env python
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
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

# waiting value between each motor rotation sequence
vitesse = 0.001 #waiting 
# waiting after openning 
waitingmona = 120

# define board value
etape = []
# define global number for increment photos taken
num = 0

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 16
camera.vflip = True
camera.hflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)
  
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

ap.add_argument("-i", "--info", 
	default="/home/pi/camera/photos/info.info",
	help="file info pixel count")

ap.add_argument("-c", "--cascade",
	default="/home/pi/cat-face-detector/result/BestMonaTop/cascade1.xml",
	help="path to cat detector haar cascade")
args = vars(ap.parse_args())

detector = cv2.CascadeClassifier(args["cascade"])

fileinfo = open(args["info"], "w")

#  define rotation sequences of plate
def rotation_plaque(seq) :
    if seq == 2 : # clockwise
      nombre_etape = 8
      etape = range(0,nombre_etape)
      etape[0] = [1,0,0,0]
      etape[1] = [1,1,0,0]
      etape[2] = [0,1,0,0]
      etape[3] = [0,1,1,0]
      etape[4] = [0,0,1,0]
      etape[5] = [0,0,1,1]
      etape[6] = [0,0,0,1]
      etape[7] = [1,0,0,1]
    if seq == 1 : # anti-clockwise
      nombre_etape = 8
      etape = range(0,nombre_etape)
      etape[0] = [0,0,0,1]
      etape[1] = [0,0,1,1]
      etape[2] = [0,0,1,0]
      etape[3] = [0,1,1,0]
      etape[4] = [0,1,0,0]
      etape[5] = [1,1,0,0]
      etape[6] = [1,0,0,0]
      etape[7] = [1,0,0,1]
    global pas
    for step in range(0,nombre_etape) :
        for pin in range(0,4) :
            port_plaque = pin_plaque[pin]
            if etape[step][pin] != 0 :
                GPIO.output(port_plaque,GPIO.HIGH)
                pas += 1
                time.sleep(vitesse)
            else :
                GPIO.output(port_plaque,GPIO.LOW)
                pas += 1
                time.sleep(vitesse)
    for pin in pin_plaque :
	GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

#  define rotation sequences of jaw
def rotation_machoir(seq) :
    if seq == 1 :# anti-clockwise
		nombre_etape = 8
		etape = range(0,nombre_etape) 
		etape[0] = [0,0,0,1]
		etape[1] = [0,0,1,1]
		etape[2] = [0,0,1,0]
		etape[3] = [0,1,1,0]
		etape[4] = [0,1,0,0]
		etape[5] = [1,1,0,0]
		etape[6] = [1,0,0,0]
		etape[7] = [1,0,0,1]
      
    if seq == 2 : # clockwise
      nombre_etape = 8
      etape = range(0,nombre_etape)
      etape[0] = [1,0,0,0]
      etape[1] = [1,1,0,0]
      etape[2] = [0,1,0,0]
      etape[3] = [0,1,1,0]
      etape[4] = [0,0,1,0]
      etape[5] = [0,0,1,1]
      etape[6] = [0,0,0,1]
      etape[7] = [1,0,0,1]
    global pas
    for step in range(0,nombre_etape) :
        for pin in range(0,4) :
            port_machoir = pin_machoir[pin]
            if etape[step][pin] != 0 :
                GPIO.output(port_machoir,GPIO.HIGH)
                pas += 1
                time.sleep(vitesse)
            else :
                GPIO.output(port_machoir,GPIO.LOW)
                pas += 1
                time.sleep(vitesse)
    for pin in pin_machoir :
	GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

# define number of step for opening 	
def openning():
	seq = 2 
	global pas
	pas = 0
	while pas < 4400 :
		rotation_machoir(seq)
	time.sleep(0.001)
	pas = 0
	while pas < 2000 :
		rotation_plaque(seq)
		rotation_machoir(seq)
	time.sleep(0.001)
	pas = 0
	while pas < 3550:
		rotation_plaque(seq)
	
# define number of step for closing 	
def closing():
	seq = 1 
	global pas
	pas = 0
	while pas < 2000 :
		rotation_machoir(seq)
		rotation_plaque(seq)
	time.sleep(0.001)
	while GPIO.input(bp) == 1 :
		rotation_plaque(seq)
	time.sleep(0.001)
	pas = 0
	while pas < 4400 :
		rotation_machoir(seq)

		
def detect():
	global num
	loop = 0
	loop2 = 0
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		rects = detector.detectMultiScale(gray, scaleFactor=1.35, minNeighbors=20, minSize=(75, 75))
		# if detect
		if rects < "[":		
			loop = loop + 1
			print "detected"
			rawCapture.truncate(0)
		# otherwise increment loop value
		else :
			loop2 = loop2 + 1
			loop = 0
			print "not detected"
			rawCapture.truncate(0)
		# if detect complet: take the framing of detection (rects[0]) , convert to black and white and count the number of black pixel
		if loop == 1:
			x, y, w, h = rects[0] 
			img = image[y:(y+h),x:(x+w)] 
			num += 1
			nb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			ret,thresh = cv2.threshold(nb,127,255, cv2.THRESH_BINARY)
			blackp = cv2.countNonZero(thresh)
			imgpath = ("/home/pi/camera/photos/imageok%s.png"%num)
			imgnb = ("/home/pi/camera/photos/nb%s.png"%num)
			# Save image and write black pixel info
			cv2.imwrite(imgpath ,image)
			cv2.imwrite(imgnb ,nb)
			infotxt =  '%s %s' % (num,blackp)
			fileinfo.write('\n')
			fileinfo.write(infotxt)
			rawCapture.truncate(0)
			return True
			break
		if loop2 == 50:
			return False
			break

def detectopen():
	loop = 0
	loop2 = 0
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		rects = detector.detectMultiScale(gray, scaleFactor=1.3,
			minNeighbors=18, minSize=(60, 60))
		rawCapture.truncate(0)
		# if detect
		if rects < "[" :		
			x, y, w, h = rects[0]
			img = image[y:(y+h),x:(x+w)]
			# Save capture
			imgpath = ("/home/pi/camera/photosopenning/imageok%s.png"%num)
			cv2.imwrite(imgpath ,img)
			loop = 1
			#print "detect"
		else:
			loop2 = loop2+1
			loop = 0
			#print "not detecded"
			rawCapture.truncate(0)
		if loop == 1 :
			loop = 0
			loop2 = 0
			rawCapture.truncate(0)
		if loop2 == 50:
			return True
			break

# define GPIO state
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
	for pin in pin_plaque :
		GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
	for pin in pin_machoir :
		GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
	GPIO.setup(capteur,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(led,GPIO.OUT)
	GPIO.setup(bp,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(bpmanuel,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	

if __name__ == '__main__':
	try:
		setup()
		ir()
		while True:
			#print('Start!')
			time.sleep(0.01) #avoid inflating the CPU
			if GPIO.input(capteur) == 0:
				light()
				if detect():
					lightoff()
					#print "opening"
					openning()
					# print "waiting during openning"
					time.sleep(waitingmona)
					#print "start detect during"
					light()
					detectopen()
					closing()
				lightoff()
			if GPIO.input(bpmanuel) == 0:
				openning()
				while GPIO.input(bpmanuel) == 1:
					time.sleep(0.01)			
				closing()

	except KeyboardInterrupt:
		GPIO.cleanup()
		

	
