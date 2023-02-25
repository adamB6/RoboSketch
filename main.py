#!/usr/bin/env python3

"""
Possibly useful information:

From the Robotis website:

Maximum translational velocity 	0.22 m/s or (1m / 4.55s)
Maximum rotational velocity 	2.84 rad/s (162.72 deg/s)
Maximum payload 	15kg
Size (L x W x H) 	138mm x 178mm x 192mm
Weight (+ SBC + Battery + Sensors) 	1kg
Threshold of climbing 	10 mm or lower
Expected operating time 	2h 30m
Expected charging time 	2h 30m

"""

import rospy
import sys
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi, degrees, radians


def process_odom(data):
    rospy.loginfo(data.pose.pose.postion.x, data.pose.pose.positon.y)


def create_line(length):
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = .22
    move_time = length/0.11  # Kinematic equation. time = distance / (1/2 Velocity)
    start = time.time()
    stop = time.time()
    while stop - start < move_time:
        process_odom()
        pub.publish(vel)
        stop = time.time()
    vel.linear.x = 0
    pub.publish(vel)
    rate.sleep()


def main():
    rospy.init_mode("sketchy_bot", anonymous=True)
    sub = rospy.Subscriber("/odom", Odometry, process_odom)
    try:
        create_line(float(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
