from picamera2 import Picamera2
from mv_shrimp import find_shrimp_features
import cv2

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})