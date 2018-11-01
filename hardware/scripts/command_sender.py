#!/usr/bin/env python

import rospy
from time import time,sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16

robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)

def move_backward():
    robot_movement_pub.publish(Point(40, -90, 0))


def move_forward():
    robot_movement_pub.publish(Point(20, 90, 0))


def move_forward2():
    robot_movement_pub.publish(Point(25, 90, 0))


def rotating():
    robot_movement_pub.publish(Point(0, 0, 40))


def stop():
    robot_movement_pub.publish(Point(0, 0, 0))


def rounding():
    robot_movement_pub.publish(Point(10, 0, 20))


def thrower(speed):
    thrower_pub.publish(Int16(speed))


def command_sender(cmd):
    rospy.init_node('command_sender', anonymous=True)
    rate = rospy.Rate(17)
    while not rospy.is_shutdown():
        # VALUES ARE BETWEEN 1000 AND 2020 / 8
        thrower(240)
        rate.sleep()


if __name__ == '__main__':
    try:
        command_sender("")
    except rospy.ROSInterruptException:
        pass