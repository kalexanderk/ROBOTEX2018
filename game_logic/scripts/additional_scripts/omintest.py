import rospy
from time import time, sleep
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int16

center_thrower = 660


class omnitest():

    def __init__(self):
        # отримуємо повідомлення про обробку зображення
        self.image_processing = rospy.Subscriber("image_processing/objects",
                                                 String, self.new_object_callback_objects)
        # надсилаємо запити на запуск моторів та thrower'а
        self.robot_movement_pub = rospy.Publisher('robot_movement', Point, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower", Int16, queue_size=10)

        self.ball_x = None
        self.ball_y = None
        self.basket_x = None
        self.basket_y = None
        self.basket_dist = None

        self.state = 1
        # state 0 - wait for XBEE command
        # state 1 - look for the ball and center
        # state 2 - approach the ball
        # state 3 - center for the basket
        # state 4 - charge the ball towards the basket

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

    def approch(self):
        self.robot_movement_pub.publish(Point(25, 90, 0))

    def rotatingR(self):
        self.robot_movement_pub.publish(Point(0, 0, 40))

    def rotatingL(self):
        self.robot_movement_pub.publish(Point(0, 0, -40))

    def stop(self):
        self.robot_movement_pub.publish(Point(0, 0, 0))

    def roundingR(self):
        self.robot_movement_pub.publish(Point(10, 0, 20))

    def roundingL(self):
        self.robot_movement_pub.publish(Point(10, 0, -20))

    def thrower(self, speed):
        self.thrower_pub.publish(Int16(speed))

    def diagonalFR(self):
        self.robot_movement_pub.publish(Point(10, 45, 0))
    def diagonalFR(self):
        self.robot_movement_pub.publish(Point(10, 45, 0))


if __name__ == "__main__":

    rospy.init_node('game_logic_node', anonymous=True)
    rate = rospy.Rate(60)

    game_logic = omnitest()
    print("Rounding Right")
    while not rospy.is_shutdown():
        if game_logic.state == 1:
            game_logic.roundingR()
            sleep(1)
            game_logic.state = 2
            print("Rounding Left")
        elif game_logic == 2:
            game_logic.rotatingL()
            sleep(1)
            game_logic.state = 3
            print("Going Diagonal Front Right")
        elif game_logic == 3:
            game_logic.diagonalFR()
            sleep(1)
            game_logic.state = 1
            print("Rounding Right")


