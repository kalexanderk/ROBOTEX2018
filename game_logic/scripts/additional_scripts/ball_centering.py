#!/usr/bin/env python
# ball_centering.py -- Tries to center the biggest (currently the only one) ball in the camera
# (horizontally)

import rospy
import math
from hardware.comport_mainboard import ComportMainboard
from std_msgs.msg import String


WHEEL1 = 60
WHEEL2 = 300
WHEEL3 = 180

def wheels_move(direction_deg, speed, angular_velocity=0):

    V1 = speed * math.sin(math.radians(WHEEL1 - direction_deg))  # |
    V2 = speed * math.sin(math.radians(WHEEL2 - direction_deg))  # 3
    V3 = speed * math.sin(math.radians(WHEEL3 - direction_deg))

    V1 += angular_velocity
    V2 += angular_velocity
    V3 += angular_velocity

    return (int(round(V1)), int(round(V2)), int(round(V3)))

def callback_function(message):
    global stop_f
    position = message.data.split("\n")[0]
    # #print(position)
    if position == "None" and stop_f == False:
        coords = wheels_move(0, 0, -5)
        mboard.launch_wheel_motors(*coords)
    elif position != "None":
        x = float(position.split(";")[0])
        print(x)
        if x < 610 and stop_f == False:
         coords = wheels_move(0, 0, 5)
         mboard.launch_wheel_motors(*coords)
        elif x > 670 and stop_f==False:
            coords = wheels_move(0, 0, 5)

            mboard.launch_wheel_motors(*coords)
        else:
            stop_f = True
            print("stop")
            coords = wheels_move(0, 0)
            mboard.launch_wheel_motors(*coords)


if __name__ == "__main__":
    mboard = ComportMainboard()
    mboard.run()
    #mboard.launch_thrower(1200)
    stop_f = False







    rospy.init_node("ball_centering")
    rospy.Subscriber("image_processing/objects", String, callback_function)
    rospy.spin()
