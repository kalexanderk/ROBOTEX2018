
#! /usr/bin/env python
import rospy
from time import time,sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16

class GameLogic():
    def __init__(self):
        #отримуємо повідомлення про обробку зображення
        self.image_processing = rospy.Subscriber("image_processing/objects", String, self.new_object_callback_ball)
        #надсилаємо запити на запуск моторів та thrower'а
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)

        self.ball_x = None
        self.ball_y = None
        self.basket_x = None
        self.basket_y = None

    def new_object_callback_ball(self, message):
        position = message.data.split("\n")[0]
        self.ball_x = float(position.split(";")[0])
        self.ball_y = float(position.split(";")[1])

    def new_object_callback_basket(self, message):
        position = message.data.split("\n")[1]
        self.ball_x = float(position.split(";")[0])
        self.ball_y = float(position.split(";")[1])

    # маємо умову, що м'яч має знаходитися за 10 чи менше см від робота посередині. Лише потім ми його хапаємо і
    # кидаємо до кошика
    def approach_and_throw_ball(self):
        ball_center = 275
        basket_center = 300 #WE HAVE @ FRAMES FOR BASKET AND FOR BALL; WE DEFINE OUR COLOR FOR BASKET IN IMAGE_PROCESSING
        if self.ball_x is None:
            self.rotating()
        #зупинилися на певній відстані від м'яча, далі маючи м'яч в центрі робимо коловий рух навколо нього доти,
        #доки кошик не опиниться в центрі теж. опісля рухаємося повільно до м'яча (щоп'ять секунд пеервіряючи близькість
        #до м'яча) із увімкненим thrower'ом і таким чином стріляємо до кошика (поки не визначили з якою швидкістю обрертати
        #thrower)
        elif (self.ball_y > 160 and self.ball_y < 180): #CALIBRATE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            self.rounding()
            print('do rounding')
            if (self.basket_x < basket_center + 20 and self.basket_x > basket_center - 20): #CALIBRATE!!!!!!!!!!!!!!!!!!!!!!!!!!
                self.thrower(1800)
                stopping_time = time()
                while (time()-stopping_time< 5):
                    print('I am moving forward')
                    self.move_forward2()
                    sleep(0.5)
                    print('I am stopping')
                self.stop()

                sleep(10)

        #обертаємося доки м'яч не опиниться посередині фрейму (який для м'яча)
        elif (self.ball_x < ball_center - 20 and self.ball_x > ball_center + 20):
            self.rotating()

        #рухаємося до м'яча як тільки він опиниться посередині фрейму
        elif (self.ball_x < ball_center + 20 and self.ball_x > ball_center - 20):
            self.move_forward()

        #інакше просто обертаємося
        else:
            self.rotating()

    def move_backward(self):
        self.robot_movement_pub.publish(Point(40, -90, 0))

    def move_forward(self):
        self.robot_movement_pub.publish(Point(20, 90, 0))

    def move_forward2(self):
        self.robot_movement_pub.publish(Point(25, 90, 0))

    def rotating(self):
        self.robot_movement_pub.publish(Point(0, 0, 40))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def rounding(self):
        self.robot_movement_pub.publish(Point(10, 0, 20))

    def thrower(self, speed):
        self.thrower_pub.publish(Int16(speed))


if __name__ == "__main__":
    rospy.init_node('game_logic_node', anonymous=True)
    rate = rospy.Rate(17)

    game_logic = GameLogic()

    while not rospy.is_shutdown():
        game_logic.approach_and_throw_ball()
        rate.sleep()