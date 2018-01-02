#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from crui.controller import CRUIController


def main():
    rospy.init_node('crui_manager')
    moveit_commander.roscpp_initialize(sys.argv)
    _ = CRUIController()

    try:
        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown(reason="Interrupted")


if __name__ == '__main__':
    main()
