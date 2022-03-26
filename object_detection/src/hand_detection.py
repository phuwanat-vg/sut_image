#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image


class faceDetect(object):

    def __init__(self):
        self.bridge = CvBridge()
        
        self.point = Vector3()
        self.face_cascade = cv2.CascadeClassifier('/home/vm20/robot_ws/src/sut_image/object_detection/src/haarcascade_frontalface_default.xml')
        self.img = None
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image, self.camera_callback)
        print(" << Subscribe image from camera")
        self.point_pub = rospy.Publisher("face_position", Vector3, queue_size = 5)
        print(" >> Publish sign status ")

    def camera_callback(self,data):
    	try:
    		img=self.bridge.imgmsg_to_cv2(data,"bgr8")
    	except CvBridgeError as e:
    		print(e)
    	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    	faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
    	# Draw the rectangle around each face
    	for (x, y, w, h) in faces:
    		cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
    		center_x = int((x + (x+w))/2.0)
    		center_y = int((y + (y+h))/2.0)
    		self.point.x = float(center_x)
    		self.point.y = float(center_y)
    		
    		cv2.circle(img, (center_x,center_y), radius=5, color=(0, 0, 255), thickness=-1)
    		
    	
    	print(self.point)
    	
    	self.point_pub.publish(self.point)
    	
    	cv2.imshow("Image Window",img)
    	cv2.waitKey(20)

def main():
     
     rospy.init_node("sign_detection_node", anonymous=True)
     fd=faceDetect()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        
#https://github.com/EshginGuluzade/stop_sign_detection/blob/main/stop_sign_detection.py

