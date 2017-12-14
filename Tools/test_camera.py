from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import RPi.GPIO as GPIO


# Pin in use
pinlight = 5
pinlight2 = 21

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)


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
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp
			# and occupied/unoccupied text
			image = frame.array

			# show the frame
			imgsave = image
			imgpath = ("/home/pi/photos/imageok.png")
			cv2.imwrite(imgpath ,imgsave)
			print "boucle"
			# clear the stream in preparation for the next frame
			rawCapture.truncate(0)

				

	except KeyboardInterrupt:
		GPIO.cleanup()
		
		
GPIO.cleanup()

