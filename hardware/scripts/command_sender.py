#!/usr/bin/env python

import rospy
import hardware.comport_mainboard as mainboard

def command_sender(cmd):
    rospy.init_node('command_sender', anonymous=True)
    mboard = mainboard.ComportMainboard()
    mboard.run()
    rate = rospy.Rate(10)
    print("before")
    while not rospy.is_shutdown():
        # mboard.write(cmd)
        mboard.launch_wheel_motors(-20, 20, 20)
        # mboard.launch_thrower(1500)
        # mboard.get_wheel_speeds()
        #
        print("after")
        rate.sleep()


if __name__ == '__main__':
    try:
        command_sender("")
    except rospy.ROSInterruptException:
        pass