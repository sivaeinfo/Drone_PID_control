#!/usr/bin/env python


from pyzbar import pyzbar
import cv2
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class qr_code_detector:
    def __init__(self):
        rospy.init_node('qr_code_detector') 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image,  self.callback)
        self.object_pub = rospy.Publisher("object_detection", String, queue_size=1)
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        self.target=[0,1,2]


    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
 
        # find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(cv_image)
       
        # loop through all the possible barcodes within images
        for barcode in barcodes:
            barcodeData = barcode.data.decode("utf-8")
            self.image_pub.publish( self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.object_pub.publish(barcodeData)



       # print(barcodeData)
            barcode_type = barcode.type
            print(barcode_type)

         

            dummy=[barcodeData]
              #print(dummy)
              #print(type(barcode_data))


            bubble_gum = dummy[0].split(',')
            unnaku_thevayana_value = map(float,bubble_gum)
            print(unnaku_thevayana_value)

            self.target[0]=unnaku_thevayana_value[0]
            self.target[1]=unnaku_thevayana_value[1]
            self.target[2]=unnaku_thevayana_value[2]

        
            if barcode_type== "QRCODE":
                print(self.target)



def main(): 
    qrd = qr_code_detector()
    #rospy.init_node('qr_code_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

 
if __name__ == '__main__':
    main()