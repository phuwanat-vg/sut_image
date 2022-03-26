#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3

face_position = Vector3()

def main():
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	face_status_sub = rospy.Subscriber('face_position', Vector3, face_callback)
	left_bound = 120
	right_bound = 200 
	vel_msg = Twist()
	v = 0.0
	w = 0.0
	rospy.init_node('face_commander_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		if face_position.x > right_bound:
			v = 0.0
			w = -0.3
		elif face_position.x < left_bound:
			v = 0.0
			w = 0.3
		else:
			w = 0.0
			w = 0.0
		vel_msg.linear.x = v
		vel_msg.angular.z = w
		vel_pub.publish(vel_msg)
		rate.sleep()
        
def face_callback(position_in):
	global face_position
	face_position = position_in

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
