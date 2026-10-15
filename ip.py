# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import ImageIJToXY
import numpy as np
from matplotlib import pyplot as plt
import math
import numpy
from time import sleep

def LogicIdentifyGarbage(cvImage):
        print("[Info :] Started detecting garbage...")
        garbagePoses = []
        resize = cvImage
        rgb = cv2.split(cvImage)
        sel = rgb[2]
        eq = cv2.equalizeHist(sel)
        #cv2.imshow("equalisation", eq)
        histr = cv2.calcHist([eq],[0],None,[256],[0,256])
        plt.plot(histr,color = 'r')
        plt.xlim([0, 256])
        minValuePixelValue, val = numpy.where( histr == histr.min())
        minValue = 190#numpy.average(minValuePixelValue)
        ret, garbagePixelsMask = cv2.threshold(sel, minValue, 255, cv2.THRESH_BINARY)
        garbagePixels = cv2.bitwise_and(resize, resize , mask = garbagePixelsMask)
        #cv2.imshow("Garbage Pixels", garbagePixels)
        contours, hierarchy = cv2.findContours(garbagePixelsMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # loop over the contours
        for c in contours:
            # compute the center of the contour
            if(len(c) > 30):
                     M = cv2.moments(c)
                     if(M["m00"] > 0):
                         cX = int(M["m10"] / M["m00"])
                         cY = int(M["m01"] / M["m00"])
                         cv2.drawContours(cvImage, [c], -1, (0, 255, 0), 2)
                         cv2.circle(resize, (cX, cY), 7, (255, 255, 255), -1)
                         rows,cols = resize.shape[:2]
                         [vx,vy,x,y] = cv2.fitLine(c, cv2.cv.CV_DIST_L2,0,0.01,0.01)
                         lefty = int((-x*vy/vx) + y)
                         righty = int(((cols-x)*vy/vx)+y)
                         cv2.line(resize,(cols-1,righty),(0,lefty),(0,255,0),2)
                         tan=math.degrees(math.atan2(y-vy, x-vx))
                         cv2.putText(resize, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                         #X,Y = self.IJToXY.getPose(cX, cY)
                         garbagePoses.append([0, 0, tan])
        return resize
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	image=LogicIdentifyGarbage(image)

	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
