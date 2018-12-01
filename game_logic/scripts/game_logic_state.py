#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import time,sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16
import math
import numpy as np
from scipy import interpolate
center_thrower = 660

class GameLogicState():

    def __init__(self):

        '''TODO: HOW IS IT DEFINED FOR OUR ROBOT?!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'''
        # self.ID[0] - field number (A or B)
        # self.ID[1] - robot number (X, A, B, C or D)
        self.ID = 'BB'
        #publisher for sending the field number to image processing node
        self.field_num_pub = rospy.Publisher("field_number", String, queue_size=120)

        # getting the detected objects coordinates
        self.image_processing = rospy.Subscriber("image_processing/objects",
                                                 String, self.new_object_callback_objects)

        self.xbee_sub = rospy.Subscriber("xbee_commands", String, self.new_xbee_callback)
        self.xbee_send_pub = rospy.Publisher("xbee_send", String, queue_size=120)
        # sending requests for running motors and thrower
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)
        self.servos_pub = rospy.Publisher("servos", Point, queue_size=10)


        self.xbee = None

        self.ball_x = None
        self.ball_y = None
        self.ball_dist = None
        self.basket_x = None
        self.basket_y = None
        self.basket_dist = None

        '''Defining the functions for throw operation'''
        self.thrower_values_interpolation()

        # the integral error and previous error must be 0 at initialization of PID controller
        self.xIe = 0
        self.previousX_e = 0
        self.dIe = 0
        self.previousD_e = 0

        self.state = 0
        #state 0 - wait for XBEE command
        #state 1 - look for the ball and center
        #state 2 - approach the ball
        #state 3 - center for the basket
        #state 4 - charge the ball towards the basket

        self.servos(0,0)

    '''All the calculations below are based on the measurements during experiments'''
    def thrower_values_interpolation(self):
        angle = list(np.repeat(720, 21)) + list(np.repeat(730, 4)) + [740]
        throw_value = list(np.repeat(42, 2)) + list(np.repeat(45, 4)) + list(np.repeat(46, 2)) + \
                      [47, 49, 51, 53, 56, 58, 60, 63, 65, 65, 65, 66, 66, 71, 71, 73, 74, 76]
        dist = [0.32, 0.4, 0.5, 0.61, 0.71, 0.81, 0.91, 0.99, 1.1, 1.18, 1.29, 1.39, 1.48, 1.59,
                1.685, 1.77, 1.88, 1.99, 2.07, 2.15, 2.24, 2.37, 2.42, 2.53, 2.65, 2.83]
        '''Interpolation using cubic splines'''
        self.f_angle = interpolate.interp1d(dist, angle, kind="cubic")
        self.f_power = interpolate.interp1d(dist, throw_value, kind="cubic")


    def new_object_callback_objects(self, message):
        position_ball = message.data.split("\n")[0]
        if position_ball != "None":
            self.ball_x = float(position_ball.split(";")[0])
            self.ball_y = float(position_ball.split(";")[1])
            self.ball_dist = float(position_ball.split(";")[2])

        else:
            self.ball_x = None
            self.ball_y = None
            self.ball_dist = None

        position_basket = message.data.split("\n")[1]
        if position_basket != "None":
            self.basket_x = float(position_basket.split(";")[0])
            self.basket_y = float(position_basket.split(";")[1])
            self.basket_dist = float(position_basket.split(";")[2])
        else:
            self.basket_x = None
            self.basket_y = None
            self.basket_dist = None

    #in HTERM the Baud rate should be 9600. We are sending ASCII commands.
    def new_xbee_callback(self, message):
        received = str(message).strip('data: "n\<>-').split(":")
        if received[0] == 'ref':
            addid = received[1][1:3]
            if addid == self.ID:
                cmd = received[1][3:]
                # the robot must respond to all commands that are sent to this specific robot
                #TODO: ADD CHECKING WHETHER IT'S SENT TO OUR ROBOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                if cmd == 'START':
                    self.xbee_send_pub.publish('a'+str(self.ID)+'ACK------')
                    self.state = 1
                elif cmd == 'STOP':
                    self.xbee_send_pub.publish('a' + str(self.ID) + 'ACK------')
                    self.state = 0
                elif cmd == 'PING':
                    self.xbee_send_pub.publish('a'+str(self.ID)+'ACK------')

    # speed range for motors is ...
    def move_forward_to_ball(self):
        self.robot_movement_pub.publish(Point(25, 90, 0))

    def rotatingR(self):
        self.robot_movement_pub.publish(Point(0, 0, 40))

    def rotatingL(self):
        self.robot_movement_pub.publish(Point(0, 0, -40))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def roundingL(self):
        self.robot_movement_pub.publish(Point(12, 10, 51))

    def roundingR(self):
        self.robot_movement_pub.publish(Point(12, 170, -51))

    # values are from 125 to 250
    def thrower(self, speed):
        self.thrower_pub.publish(Int16(speed))

    # launch servos(1: 720-825; 2: <700 - continuous mode; 2300 -> 100 - fast movement)
    def servos(self, speed1, speed2):
        self.servos_pub.publish(Point(speed1, speed2, 0))
	
    # Power from 0 to 125, angle from 720-835
    def throw(self, power, angle):
        global counter_t
        global flag_t
        if flag_t == 0:
            self.thrower(power+125)
            if counter_t < 0 or counter_t > 80:
                counter_t = 0
            elif counter_t == 0:
                self.servos(angle, 0)
                print("angle set")
                counter_t += 1
            elif counter_t == 10:
                print("sleep done")
                self.servos(angle, 4000)
                counter_t += 1
            elif counter_t == 80:
                self.servos(angle, 0)
                #counter_t = 0
                flag_t = 1
            else:
                counter_t += 1
            print(counter_t)

    def theorem_Pifagor(self, slope_distance, camera_height=0.29):
        return math.sqrt(slope_distance**2 - camera_height**2) - 0.087 - 0.162


    '''PID CONTROLLER FOR THE ANGULAR SPEED (DEPENDS ON THE X COORDINATE OF THE BALL CENTER)'''
    # 710 is the desirable value for self.ball_x to become while approaching the tracked ball - 12 cm from robot
    def xPID(self):
        # current error
        currentX_e = 710 - self.ball_x

        self.xIe += currentX_e
        xDe = (currentX_e - self.previousX_e)

        # TODO: CORRECT CONSTANTS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        u = (currentX_e/4.5) + (self.xIe/600) + (xDe/10)

        self.previousX_e = currentX_e

        # TODO: DEFINE THE MAXIMUM VALUE FOR THE MOTOR AND FOR HERE RESPECTIVELY!!!!!!!!!!!!
        if math.fabs(u) > 800:
            self.xIe = 0

        return int(u)

    '''PID CONTROLLER FOR THE LINEAR SPEED (DEPENDS ON A DISTANCE TO THE BALL)'''
    # 0.1 is the desirable value for self.ball_dist to become while approaching the tracked ball
    def dPID(self):
        # current error
        currentD_e = self.ball_dist - 0.1


        self.dIe += currentD_e
        dDe = (currentD_e - self.previousD_e)

        # TODO: FIND APPROPRIATE CONSTANTS!!!!!!!!!!!!!!!!!!!!!!!!!!!
        u = (currentD_e / 10) + (self.dIe / 10) + (dDe / 10)

        self.previousD_e = currentD_e

        if math.fabs(u) > 50:
            self.dIe = 0

        return int(u)

    '''MOVE FORWARD TO TRACK THE BALL USING PID CONTROLLERS FUNCTION'''
    def move_forwardPID(self):
        pido = self.xPID()
        if pido == 0:
            self.robot_movement_pub.publish(Point(40, 90, 0))
        else:
            self.robot_movement_pub.publish(Point(40, 90, -1*pido))



if __name__ == "__main__":
    
    global counter_t
    counter_t = 0
    global flag_t
    flag_t = 0
    rospy.init_node('game_logic', anonymous=True)
    rate = rospy.Rate(25)

    game_logic = GameLogicState()

    game_logic.state = 1

    sleep(1)

    while not rospy.is_shutdown():
        # send the field number to image processing node

        game_logic.field_num_pub.publish(game_logic.ID[0])

        #mes = input().strip()

        #if mes != None and flag_t == 0:
        game_logic.throw(game_logic.f_power(game_logic.basket_dist), game_logic.f_angle(game_logic.basket_dist))
        #if flag_t == 1:
        #    mes = None
        #    flag_t = 0

        #
        # if game_logic.state == 1:
        #     print('State 1')
        #     if game_logic.ball_x != None:
        #         game_logic.state = 2
        #         print("Found the ball.")
        #     else:
        #         game_logic.rotatingR()
        #
        # elif game_logic.state == 2:
        #     print('State 2')
        #     if game_logic.ball_y > 710:
        #         game_logic.xIe = 0
        #         game_logic.state = 3
        #         print("Close to the ball.")
        #         counter_t = 0
        #         pass
        #
        #     if game_logic.ball_x is None:
        #         game_logic.xIe = 0
        #         game_logic.state = 1
        #     else:
        #         game_logic.move_forwardPID()
        #
        # elif game_logic.state == 3:
        #     print('State 3')
        #
        #     if game_logic.basket_x is None:
        #         game_logic.roundingR()
        #     elif math.fabs(game_logic.basket_x - center_thrower) < 15:
        #         game_logic.state = 4
        #         print("Found the basket; x = " + str(game_logic.basket_x))
        #     elif game_logic.basket_x >= center_thrower + 15:
        #         game_logic.roundingL()
        #
        #
        # elif game_logic.state == 4:
        #     game_logic.throw(100, 720)
        #     if flag_t == 1:
        #         game_logic.state = 1
        #     flag_t = 0

        rate.sleep()
