#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import time,sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16

class GameLogicState():

    def __init__(self):
        #отримуємо повідомлення про обробку зображення
        self.image_processing = rospy.Subscriber("image_processing/objects",
                                                 String, self.new_object_callback_objects)
        #надсилаємо запити на запуск моторів та thrower'а
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)

        self.ball_x = None
        self.ball_y = None
        self.basket_x = None
        self.basket_y = None
        self.basket_dist = None

        self.state = 1
        #state 0 - wait for XBEE command
        #state 1 - look for the ball and center
        #state 2 - approach the ball
        #state 3 - center for the basket
        #state 4 - charge the ball towards the basket

    def new_object_callback_objects(self, message):
        print(message)
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

        print("\n", self.ball_x, self.ball_y, self.basket_x, self.basket_y, self.basket_dist)
        print("============================================")



    def move_backward(self):
        self.robot_movement_pub.publish(Point(40, -90, 0))

    def move_forward(self):
        self.robot_movement_pub.publish(Point(20, 90, 0))

    def move_forward2(self):
        self.robot_movement_pub.publish(Point(25, 90, 0))

    def rotatingR(self):
        self.robot_movement_pub.publish(Point(0, 0, 40))

    def rotatingL(self):
        self.robot_movement_pub.publish(Point(0, 0, -40))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def roundingR(self):
        self.robot_movement_pub.publish(Point(10, 0, 25))

    def roundingL(self):
        self.robot_movement_pub.publish(Point(10, 0, -25))

    def thrower(self, speed):
        self.thrower_pub.publish(Int16(speed))


if __name__ == "__main__":
    rospy.init_node('game_logic_node', anonymous=True)
    rate = rospy.Rate(60)

    game_logic = GameLogicState()

    while not rospy.is_shutdown():
    
        if game_logic.state == 1:
            if game_logic.ball_x < 650 and game_logic.ball_x > 630:
                game_logic.state = 2
                print("Found da Ball")
            elif game_logic.ball_x >= 650:
                game_logic.rotatingR()
            else:
                game_logic.rotatingL()
        elif game_logic.state == 2:

            if game_logic.ball_y > 660:
                game_logic.state = 3
                print("Got to Da BALL")
                continue

            game_logic.move_forward2()
            sleep(0.5)

            if not (game_logic.ball_x < 650 and game_logic.ball_x > 630):
                game_logic.state = 1
        elif game_logic.state == 3:
            if  game_logic.basket_x < 650 and game_logic.basket_x > 630:
                game_logic.state = 4
                print("FOUND DA BASKET")
            elif game_logic.basket_x >= 650:
                game_logic.roundingR()
            else:
                game_logic.roundingL()

        elif game_logic.state == 4:
            game_logic.thrower(1700)
            game_logic.move_forward2()
            print("CHAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARGE")
            sleep(1)
            game_logic.state = 1


        rate.sleep()

