#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import time,sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16
import math

center_thrower = 660

class GameLogicState():

    def __init__(self):

        '''TODO: HOW IS IT DEFINED FOR OUR ROBOT?!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'''
        # self.ID[0] - field number (A or B)
        # self.ID[1] - robot number (X, A, B, C or D)
        self.ID = 'AB'

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


    '''PID CONTROLLER FOR THE ANGULAR SPEED (DEPENDS ON THE X COORDINATE OF THE BALL CENTER)'''
    # 660 is the desirable value for self.ball_x to become while approaching the tracked ball
    def xPID(self):
        # current error
        currentX_e = 660 - self.ball_x

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
    rospy.init_node('game_logic', anonymous=True)
    rate = rospy.Rate(25)

    game_logic = GameLogicState()

    game_logic.state = 0

    sleep(1)
    
    while not rospy.is_shutdown():
        # send the field number to image processing node
        game_logic.field_num_pub.publish(game_logic.ID[0])

        game_logic.servos(730, 100)

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
        #     # TODO: change the value from 640 to some other one as we have different frame for the new robot
        #     if game_logic.ball_y > 640:
        #         game_logic.xIe = 0
        #         game_logic.state = 3
        #         print("Close to the ball.")
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
        # if game_logic.basket_x is None:
        #     game_logic.roundingR()
        # elif math.fabs(game_logic.basket_x - center_thrower) < 15:
        #     game_logic.thrower(250)
        #     game_logic.state = 4
        #     print("Found the basket; x = " + str(game_logic.basket_x))
        # elif game_logic.basket_x >= center_thrower + 15:
        #     game_logic.roundingL()
        #
        # elif game_logic.state == 4:
        #     print('State 4')
        #     game_logic.thrower(250)
        #     game_logic.move_forward_to_ball()
        #     sleep(1)
        #     game_logic.state = 1

        rate.sleep()
