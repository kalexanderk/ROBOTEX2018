#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import hardware.comport_mainboard as mainboard
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, String

WHEEL_ONE_ANGLE = 120
WHEEL_TWO_ANGLE = 240
WHEEL_THREE_ANGLE = 0

WHEEL_DISTANCE_FROM_CENTER = 0.133
ROBOT_SPEED = 30
ROBOT_TURN_SPEED = 50

# WHEEL1 = 60 degrees
# WHEEL2 = 300 degrees
# WHEEL3 = 180 degrees


class SerialCommunication():
    def __init__(self):
        self.starter = False
        self.main_board = mainboard.ComportMainboard()
        self.main_board.run()
        self.started = True

        # listening to the commands from game logic node
        self.sub_movement = rospy.Subscriber("robot_movement", Point, self.new_object_callback_wheels)
        self.sub_thrower = rospy.Subscriber("thrower", Int16, self.new_object_callback_thrower)
        self.xbee_send_sub = rospy.Subscriber("xbee_send", String, self.new_xbee_send_callback)


        self.xbe_publisher = rospy.Publisher("xbe_commands", String, queue_size=120)


        '''Omnimotion starts from here'''

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0

    def get_speed_for_wheel(self, wheel_angle, drive_angle,
                            robot_speed, wheel_distance_from_center,
                            robot_angular_velocity):
        move_speed = robot_speed * math.cos(math.radians(drive_angle -
                            wheel_angle))
        turn_speed = wheel_distance_from_center * robot_angular_velocity
        return move_speed + turn_speed

    def set_wheels(self, w1, w2, w3):
        self.wheel_one_speed = w1
        self.wheel_two_speed = w2
        self.wheel_three_speed = w3
        self.main_board.launch_wheel_motors(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed)

    def set_movement(self, linear_speed, direction_degrees, angular_speed):
        w1 = self.get_speed_for_wheel(WHEEL_ONE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w2 = self.get_speed_for_wheel(WHEEL_TWO_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)
        w3 = self.get_speed_for_wheel(WHEEL_THREE_ANGLE, direction_degrees,
                                      linear_speed,
                                      WHEEL_DISTANCE_FROM_CENTER,
                                      angular_speed)

        self.set_wheels(round(w1, 0), round(w2, 0), round(w3, 0))

    '''End of the omnimotion'''


    '''Where the action happens'''
    def new_object_callback_wheels(self, point):
        self.set_movement(point.x, point.y, point.z)

    def new_object_callback_thrower(self, speed):
        self.main_board.launch_thrower(speed.data)

    '''Reading commands from a mainboard and publishing them to xbe_commands publisher'''
    def read_command(self):
        if self.started:
            self.xbe_publisher.publish(str(self.main_board.read()))

    '''Sending the answer to xbee commands'''
    def new_xbee_send_callback(self, message):
        self.main_board.send_referee_signal_string(str(message).split('"')[1])


if __name__ == '__main__':
    rospy.init_node('serial_communication', anonymous=True)
    rate = rospy.Rate(25)
    serial_communication = SerialCommunication()

    while not rospy.is_shutdown():
        serial_communication.read_command()
        rate.sleep()
