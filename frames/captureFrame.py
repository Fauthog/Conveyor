import cv2
from picamera2 import Picamera2
from datetime import datetime



def main():
	picam2 = Picamera2()

	picam2.configure(picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120}))
	#picam2.video_configuration.controls.FrameRate = 90.0
	#picam2.configure("video")	
	picam2.start()

	while True:
		frame = picam2.capture_array()
		cv2.imshow('Frame', frame)
		key = cv2.waitKey(1)
		if key == ord('q'):
			break
		elif key == ord('r'):
			now = datetime.now()
			dt = now.strftime("%Y%m%d%H%M%S")
			imstr = "frame_" + dt + ".jpg"
			cv2.imwrite(imstr, frame)
			continue
		else:
			continue
	picam2.stop()
	picam2.close()
	cv2.destroyAllWindows()		


if __name__ == "__main__":
	main()
