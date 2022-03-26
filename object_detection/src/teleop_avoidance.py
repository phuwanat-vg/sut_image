#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


sonic_range = 0.0
teleop_vel = Twist()

def main():
	vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	range_sub = rospy.Subscriber('ultrasound', Range, sonic_callback)
	teleop_sub = rospy.Subscriber('cmd_vel/teleop', Twist, teleop_callback)
	
	vel_msg = Twist()
	rospy.init_node('commander_node', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		if sonic_range > 0.3:
			v = teleop_vel.linear.x
			w = teleop_vel.angular.z
			
		else:
			v = 0.0
			w = 0.3
		
		
		vel_msg.linear.x = v
		vel_msg.angular.z = w
		vel_pub.publish(vel_msg)
		rate.sleep()
        
def teleop_callback(vel_in):
	global teleop_vel
	teleop_vel = vel_in

def sonic_callback(range_in):
	global sonic_range
	sonic_range = range_in.range
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
