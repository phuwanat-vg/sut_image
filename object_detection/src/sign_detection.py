#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image


class signDetect(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image, self.camera_callback)
        print(" << Subscribe image from camera")
        self.status_pub = rospy.Publisher("sign_detection", String, queue_size = 5)
        print(" >> Publish sign status ")
        self.st = String()
        self.stop_sign = cv2.CascadeClassifier('/home/vm20/robot_ws/src/sut_image/object_detection/src/cascade_stop_sign.xml')
        self.sign_status = ""
        self.img = None

    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        img=self.bridge.imgmsg_to_cv2(data,"bgr8")
        self.status_pub.publish(self.st)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        stop_sign_scaled = self.stop_sign.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in stop_sign_scaled:
        	# Draw rectangle around the stop sign
        	stop_sign_rectangle = cv2.rectangle(img, (x,y),(x+w, y+h),(0, 255, 0), 3)
        	stop_sign_text = cv2.putText(img=stop_sign_rectangle,
                                     text="Stop Sign",
                                     org=(x, y+h+30),
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=1, color=(0, 0, 255),
                                     thickness=2, lineType=cv2.LINE_4)
        if len(stop_sign_scaled)>0:
       		sign_status = "STOP"
       	else:
       		sign_status = "GO"
       	self.st.data  = sign_status
       	self.status_pub.publish(self.st)
      
        cv2.imshow("Image Window",img)
        cv2.waitKey(20)

def main():
     
     rospy.init_node("sign_detection_node", anonymous=True)
     sd=signDetect()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        
#https://github.com/EshginGuluzade/stop_sign_detection/blob/main/stop_sign_detection.py

