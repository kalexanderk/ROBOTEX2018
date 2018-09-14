#!/usr/bin/env python

import rospy
import hardware.comport_mainboard as mainboard

def command_sender(cmd):
    rospy.init_node('command_sender', anonymous=True)

    mboard = mainboard.ComportMainboard()
    mboard.run()

    print("before")
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print("after")
        mboard.write("b\n")
        rate.sleep()

if __name__ == '__main__':
    try:
        command_sender("b")
    except rospy.ROSInterruptException:
        pass
