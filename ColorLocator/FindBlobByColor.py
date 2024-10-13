# finds the largest blob of a speified color (hue)
# draws a bounding box and a center point)
# based on "Learn Robotics with Raspberry Pi by Matt Timmons-Brown
# modified by Chris Fischer for picamera2, openCV2
#
# import the necessary packages
#from picamera.array import PiRGBArray
#from picamera import PiCamera
from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
#camera = PiCamera()

#camera.resolution = (640, 480)
#camera.framerate = 32
#rawCapture = PiRGBArray(camera, size=(640, 480))

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start_preview(Preview.QTGL)
# more configuration from a display program
#picam2.preview_configuration.main.size = (800,600)
picam2.preview_configuration.main.format = "RGB888"
#picam2.preview_configuration.align()



picam2.start()


while True:
	#while True:
	#	try:
	#		hue_value = int(input("Hue value between 10 and 245: "))
	#		if (hue_value < 10) or (hue_value > 245):
	#			raise ValueError
	#	except ValueError:
	#		print("That isn't an integer between 10 and 245, try again")
	#	else:
	#		break
    # approximate hue values  
    #    red    150
    #    blue    10
    #    yellow  70
    #  note these hues are possibly based on swapped red and blue 
    #  channels BGR vs RGB
    #  picks the correct color, but some of the graphic display images
    #   are swapped.
    #   documentation is unclear
	
	hue_value = 150 
	
	#   alllow a range of nearby hues   Book used +/- 10   Fischer expanded to +/- 30		
	lower_hue = np.array([hue_value-30,100,0])
	upper_hue = np.array([hue_value+30, 255, 255])
	                                                                             

#	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	for i in range(0,5):
		
		#image is a numpy / libcamera format image array
		#no need to directly access it.  opencv2 provides tools to manipulate the data
		#
		image = picam2.capture_array()
		#picam2.capture_array() apparently (not sure) returns BGR - Blue Green Red format
		#open_cv2 expects RGB?   Maybe?
		#image = frame.array
		# convert BGR format to RBG
		image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

		
		#HSV hue detection seems to play better with BGR
		#hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

		color_mask = cv2.inRange(hsv, lower_hue, upper_hue)

		result = cv2.bitwise_and(image, image, mask= color_mask)

		cv2.imshow("Camera Output", image_rgb)
		cv2.imshow("HSV", hsv)
		cv2.imshow("Color Mask", color_mask)
		cv2.imshow("Final Result", result)
		#image2,contours,hierarchy = cv2.findContours(color_mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		contours,hierarchy = cv2.findContours(color_mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		objectArea = 0
		startPoint = 0
		endPoint = 0
		for contour in contours:
			x,y,w,h = cv2.boundingRect(contour)
			foundArea = w*h
			if objectArea < foundArea:
				objectArea = foundArea
				print("w = ",w," h = ", h)
				objectX = x + (w/2)
				objectY = y + (h/2)
				objectW = w
				objectH = h
		
		startPoint = (int(objectX-objectW/2),int(objectY-objectH/2))
		#startPoint = (5,5)
		endPoint = (int(objectX+objectW/2),int(objectY+objectH/2))
		#endPoint = (70,70)
		centerPoint1 = (int(objectX-2),int(objectY-2))
		centerPoint2 = (int(objectX+2),int(objectY+2))
		print("start, end ", startPoint, endPoint)
		color = (255,255,255)
		lineThickness = 2
		
		image = cv2.rectangle(image,startPoint, endPoint,color,	lineThickness)
		image = cv2.rectangle(image,centerPoint1, centerPoint2, color, 3)
		cv2.imshow("Found Object",image)
		
		con = input("any key to continue ")
		
			

		#rawCapture.truncate(0)

		k = cv2.waitKey(5) #& 0xFF
		if "q" == chr(k & 255):
			break
