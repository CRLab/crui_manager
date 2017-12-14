#!/usr/bin/env python

import rospy
import sys
import moveit_commander

from crui_helpers import CRUIController, CRUIManager


def main():
    rospy.init_node('crui_manager')
    moveit_commander.roscpp_initialize(sys.argv)
    crui_manager = CRUIManager()
    crui_controller = CRUIController(crui_manager)

    try:
        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown(reason="Interrupted")


if __name__ == '__main__':
    main()
