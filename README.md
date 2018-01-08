# Catfood

Catfood is a Python script working in a RaspberryPi 3 with Raspbian. Catfood recognizes one cat on two different cats with Opencv and opens access to the food for the recognized cat.

### Prerequisites
To recognize the right cat, you must first generate an xml file Cascade Classifier with opencv.
#### -- For example --
While the good cat was eating, I started a script detecting the cat's face and cropped the image in the center of his forehead so that he had his own marks.
```python
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		rects = detector.detectMultiScale(gray, scaleFactor=1.25,
			minNeighbors=20, minSize=(75, 75))
		if rects < "[" :		#Si detection
      x, y, w, h = rects[0] 
      pw = (w * 15)/100 #percent width
      ph = (h * 28)/100 #percent hight
      img = frame[y+pw:(y-pw)+w,x+ph:(x-ph)+h] #cropped image
```
Then with about 200 good images cropped, i could start the training of classifier with opencv_traincascade.

## Installing
#### Raspberry OS : 
 - Raspbian
#### Package :
 - Python 2.7 or 3
 - OpenCV 2
#### Materials : 
 - Raspberry Pi
 - Two steppers motors 5v with ULN2003 driver and alimention 5v
 - Raspberry Pi Night Vision Camera
 - IR Break Beam Sensor 
 - White LED Board Specific For Raspberry Pi Camera x2
 - Pushbutton x2 (one for end plate postion and one for manual opening) 
 - Wires
 
 ## Deployment
 #### Bash Script :
 The python script is start as deamon during Raspbian start, so you have to create a bash file.
 ```bash
 #!/bin/sh
### BEGIN INIT INFO
# Provides:          MonaFood
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start Python program at boot time
# Description:       Mona Food
### END INIT INFO

PATH=/usr/sbin:/usr/bin:/sbin:/bin
DESC='Startup script python monafood'
NAME=monafood
DAEMON=/usr/sbin/$NAME
PIDFILE=/var/run/$NAME.pid
SCRIPTNAME=/etc/init.d/$NAME

case "$1" in
    start)
        python /home/pi/camera/monafood.py
    ;;

    *)
         echo 'Usage: /etc/init.d/monafood {start}'
        exit 1
    ;;
esac
exit 0
 ```
##### Install
```bash
 sudo chmod 0755 /etc/init.d/monafood
 ```
 ```bash
 sudo update-rc.d monafood defaults
 ```

 
## Next Update

- The success rate is around 95%, the bad cat can be detected. The black pixels are counted and list when there is a detection. So to put a second identification with the number of black pixel counted.
```python
nb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(nb,127,255, cv2.THRESH_BINARY)
blackp = cv2.countNonZero(thresh)
```

- Problem detect during the night. The detection is longer and difficult with low light. A third source of light is to consider.




