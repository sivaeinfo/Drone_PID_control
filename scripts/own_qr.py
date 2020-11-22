#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from __future__ import unicode_literals
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import pyzbar.pyzbar as pyzbar

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		#self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()




	def image_callback(self, data):		
		self.image_sub = cv2.Videoimage_subture(0) #cam ON

		i = 0
		
		while(self.image_sub.isOpened()):#checking cam ON?OFF
		  ret, img = self.image_sub.read()# assigning the image_subtured qr value to ret and img

		  if not ret:
		    continue
		    

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #image colour convertion
		     
		decoded = pyzbar.decode(gray) #decode the scanned qr
		barcode_type=0
		barcode_data='0'


		for d in decoded: 
		    x, y, w, h = d.rect

		    barcode_data = d.data.decode("utf-8")
		    barcode_type = d.type

		    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

		    text = '%s (%s)' % (barcode_data, barcode_type)
		    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
		  
		 
		  
		  #break

		cv2.imshow('Edrone cam', Image)


		 

		dummy=[barcode_data]
		  #print(dummy)
		  #print(type(barcode_data))


		bubble_gum = dummy[0].split(',')
		unnaku_thevayana_value = map(float,bubble_gum)
		print(unnaku_thevayana_value)



	   	self.image_sub.release()
	   	cv2.destroyAllWindows()


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()