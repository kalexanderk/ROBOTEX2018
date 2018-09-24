#!/usr/bin/env python

import rospy
import hardware.comport_mainboard as mainboard

def command_sender(cmd):
    rospy.init_node('command_sender', anonymous=True)

    mboard = mainboard.ComportMainboard()
    mboard.run()

    print("before")
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        print("after")
        mboard.write(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        command_sender("sd:-20:20:0:0\n")
    except rospy.ROSInterruptException:
        pass
