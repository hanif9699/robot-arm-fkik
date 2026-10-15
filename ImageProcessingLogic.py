from threading import Thread
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import cv2
import ImageIJToXY
import numpy as np
from matplotlib import pyplot as plt
import math
import numpy
from time import sleep


class ImageProcessingLogic(Thread):
    """description of class"""

    def __init__(self, IJToXY ):
        Thread.__init__(self)
        self.Run = True
        #self.initialisePiCamera()
        #self.initialiseWebCamera()
        self.IJToXY = IJToXY
        self.garbagePoses = []

    def initialisePiCamera(self):
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
        self.image= None
        # allow the camera to warmup
        time.sleep(0.1)
        self.IsPiCamera = True

    def initialiseWebCamera(self):
        self.camera = cv2.VideoCapture(-1)
        sleep(0.1)
        self.IsPiCamera =  False
        

    def LogicIdentifyGarbage(self, cvImage):
        self.garbagePoses = []
        resize = cvImage
        rgb = cv2.split(cvImage)
        sel = rgb[0]
        #eq = cv2.equalizeHist(sel)
        #cv2.imshow("equalisation", eq)
        #histr = cv2.calcHist([eq],[0],None,[256],[0,256])
        #plt.plot(histr,color = 'r')
        #plt.xlim([0, 256])
        #minValuePixelValue, val = numpy.where( histr == histr.min())
        minValue = 240#numpy.average(minValuePixelValue)
        ret, garbagePixelsMask = cv2.threshold(sel, minValue, 255, cv2.THRESH_BINARY)
        garbagePixels = cv2.bitwise_and(resize, resize , mask = garbagePixelsMask)
        
        #cv2.imshow("Garbage Pixels", garbagePixels)
        contours, hierarchy = cv2.findContours(garbagePixelsMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # loop over the contours
        for c in contours:
            area = cv2.contourArea(c)
            # compute the center of the contour
            if(len(c) > 100 and area > 400):
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
			 #cv2.line(resize,(vx,vy),(x,y),(0,255,0),2)
                         tan=math.degrees(math.atan2(vy, vx))
                         cv2.putText(resize, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                         X,Y = self.IJToXY.getPose(cX, cY)
                         self.garbagePoses.append([X, Y, tan])
        return resize


    def ShowImage(self):
        #if(self.image is not None):
               cv2.imshow("Pi camera", self.image)
               key = cv2.waitKey(1) & 0xFF
    
    def processPiCamera(self):
         # capture frames from the camera
            co =0
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                self.image = frame.array

                if(co > 0):
                   self.image = self.LogicIdentifyGarbage(self.image)
                   co  = 0
                co = co + 1
                # show the frame
                #cv2.imshow("Pi camera", self.image)
                time.sleep(0.1)
                self.ShowImage()
                # clear the stream in preparation for the next frame
                self.rawCapture.truncate(0)

                if(self.Run == False):
                    break

    def ProcessFromWebCamera(self):
	print("[Info :] Started detecting garbage...")
        self.camera = cv2.VideoCapture(-1)
        sleep(1)
        co =0
        while(self.Run):
            ret, frame =  self.camera.read()
            if(ret):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.image = frame
                if(True):
                   self.image = self.LogicIdentifyGarbage(self.image)
                   #co  = 0
                co = co + 1
                self.ShowImage()
                # show the frame
                #cv2.imshow("Pi camera", self.image)
                if(co > 20):
                    time.sleep(1)
                    self.camera.release()
                    return
            else:
                x = 0
                #print("Frame skipped ")



    def run(self):
            print("[Info :]  Starting image processing thread to detect garbage" )
            if(self.IsPiCamera ==  False):
                self.ProcessFromWebCamera()
            else:
                self.processPiCamera()
            


    def __del__(self):
        self.Run = False
        #self.join()
        
def create():
    converter = ImageIJToXY.ImageIJToXY(np.array([0, 0]), np.array([640, 480]), np.array([-10.0, -10.0]), np.array([10.0, 10.0]))
    ip = ImageProcessingLogic(converter)
    #ip.start()
    while(True):
       ip.ProcessFromWebCamera()

    del ip
    return

if __name__ == '__main__':
    create()

            



