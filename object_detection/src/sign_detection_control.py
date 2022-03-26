#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


sonic_range = 0.0
sign_status = ""

def main():
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	range_sub = rospy.Subscriber('ultrasound', Range, sonic_callback)
	sign_status_sub = rospy.Subscriber('sign_detection', String, sign_callback)
	
	vel_msg = Twist()
	rospy.init_node('sign_commander_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		if sign_status == "GO":
			if (sonic_range > 0.3):
				v = 0.3
				w = 0.0
			else:
				v = 0.0
				w = 0.3
		else:
			v = 0.0
			w = 0.0
		
		
		vel_msg.linear.x = v
		vel_msg.angular.z = w
		vel_pub.publish(vel_msg)
		rate.sleep()
        
def sign_callback(status_in):
	global sign_status
	sign_status = status_in.data

def sonic_callback(range_in):
	global sonic_range
	sonic_range = range_in.range
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
