import cv2
from time import sleep

camera = cv2.VideoCapture(-1)
sleep(1)
co =0
while(True):
   ret, frame =  camera.read()
   if(ret):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.imshow("Pi camera", frame)
                key = cv2.waitKey(1) & 0xFF
                sleep(0.1)
