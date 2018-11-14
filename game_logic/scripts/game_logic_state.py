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


        self.xbee = None

        self.ball_x = None
        self.ball_y = None
        self.basket_x = None
        self.basket_y = None
        self.basket_dist = None

        self.xIe = 0
        self.xPe = 0

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
        else:
            self.ball_x = None
            self.ball_y = None

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

    def xPID(self):
        cEr = 660 - self.ball_x
        self.xIe += cEr
        div = cEr - self.xPe
        ret = (cEr/4.5) + (self.xIe/600) + (div/10)
        self.xPe = cEr
        if math.fabs(ret) > 800:
            self.xIe = 0
        return int(ret)

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

    game_logic.state = 1

    sleep(1)

    while not rospy.is_shutdown():

        # send the field number to image processing node
        game_logic.field_num_pub(game_logic.ID[0])

        if game_logic.state == 1:
            print('State 1')
            if game_logic.ball_x != None:
                game_logic.state = 2
                print("Found the ball.")
            else:
                game_logic.rotatingR()

        elif game_logic.state == 2:
            print('State 2')
            if game_logic.ball_y > 640:
                game_logic.xIe = 0
                game_logic.state = 3
                print("Close to the ball.")
                pass

            if game_logic.ball_x is None:
                game_logic.xIe = 0
                game_logic.state = 1
            else:
                game_logic.move_forwardPID()

        elif game_logic.state == 3:
            print('State 3')
            if math.fabs(game_logic.basket_x - center_thrower) < 15:
                game_logic.thrower(250)
                # sleep(0.5)
                game_logic.state = 4
                print("Found the basket; x = " + str(game_logic.basket_x))
            elif game_logic.basket_x >= center_thrower + 15:
                game_logic.roundingL()
            else:
                game_logic.roundingR()

        elif game_logic.state == 4:
            print('State 4')
            game_logic.thrower(250)
            game_logic.move_forward_to_ball()
            sleep(1)
            game_logic.state = 1

        rate.sleep()
