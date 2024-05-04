import time
import numpy as np
import threading
import sys
import datetime
import cv2
from picamera2 import Picamera2



class camera():
	def __init__(self):
		self.picam2 = Picamera2()
		self.picam2.configure(self.picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120}))
		#picam2.video_configuration.controls.FrameRate = 90.0
		#picam2.configure("video")
		self.picam2.start()
		self.stopcapturing:bool = False


	def captureImages(self)->None:
		while True:
			if self.stop_capturing:
				return
			img = self.picam2.capture_array()
			cv2.imshow("Live image", img)
			# char = cv2.waitKey(1)
			# if(char==ord('s')):
			# 	print("Saving an image")
			# 	cv2.imwrite("img_{}.jpg".format(time.time()), img)
 
	def startCapture(self)->None:
		self.stopcapturing=True
		self.camera_thread = threading.Thread(target=self.captureImages)
		self.camera_thread.start()

	def stopCapture(self)->None:
		self.stopcapturing=True		
		self.camera_thread.join()
		return
	 
