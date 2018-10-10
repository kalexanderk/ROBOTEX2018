#!/usr/bin/env python
import rospy
import math
import hardware.comport_mainboard as mainboard
from geometry_msgs.msg import Point
from std_msgs.msg import Int16

WHEEL_ONE_ANGLE = 120
WHEEL_TWO_ANGLE = 240
WHEEL_THREE_ANGLE = 0

WHEEL_DISTANCE_FROM_CENTER = 0.133
ROBOT_SPEED = 30
ROBOT_TURN_SPEED = 50

WHEEL1 = 60
WHEEL2 = 300
WHEEL3 = 180


class SerialCommunication():
    def __init__(self):

        #слухаємо команди від game_logic
        self.sub = rospy.Subscriber("robot_movement", Point, self.new_object_callback_wheels)
        self.sub = rospy.Subscriber("thrower", Int16, self.new_object_callback_thrower)

        self.main_board = mainboard.ComportMainboard()
        self.main_board.run()

        '''Omnimotion starts from here'''

        self.wheel_one_speed = 0
        self.wheel_two_speed = 0
        self.wheel_three_speed = 0


    # def get_wheels_speeds(direction_deg, linear_speed, angular_velocity=0):
    #     V1 = linear_speed * math.sin(math.radians(WHEEL1 - direction_deg))  # |
    #     V2 = linear_speed * math.sin(math.radians(WHEEL2 - direction_deg))  # 3
    #     V3 = linear_speed * math.sin(math.radians(WHEEL3 - direction_deg))
    #     V1 += angular_velocity
    #     V2 += angular_velocity
    #     V3 += angular_velocity
    #     return (int(round(V1)), int(round(V2)), int(round(V3)))

    ''''''
    #TRY TO USE THE ABOVE FUNCTION INSTEAD
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
        self.main_board.launch_wheel_motors(self.wheel_one_speed, self.wheel_two_speed, self.wheel_three_speed,0)

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

        self.set_wheels(round(w1, 2), round(w2, 2), round(w3, 2))
    ''''''

    '''End of the omnimotion'''

    def new_object_callback_wheels(self, point):
        self.set_movement(point.x, point.y, point.z)

    def new_object_callback_thrower(self, speed):
        self.main_board.launch_thrower(speed.data)


if __name__ == '__main__':
    rospy.init_node('serial_communication', anonymous=True)
    rate = rospy.Rate(17)  # 2Hz
    serial_communication = SerialCommunication()

    while not rospy.is_shutdown():
        rate.sleep()