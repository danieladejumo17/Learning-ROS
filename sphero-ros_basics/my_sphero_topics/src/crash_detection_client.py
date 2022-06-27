#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from cmd_vel_publisher import CmdVelPub


class CrashDetectionClient():
    def __init__(self):
        rospy.wait_for_service("detect_crash")
        rospy.loginfo("detect_crash service available!")

        self.service_proxy = rospy.ServiceProxy("detect_crash", Trigger)
        self.cmd_vel_pub = CmdVelPub()
        self.last_move_to = "forward"

    def detect(self):
        return self.service_proxy(TriggerRequest())

    def drive(self):
        detect = self.detect()
        move_to = detect.message

        if detect.success and (not move_to == "nothing"):
            # if there is a collision and a valid direction to move
            self.cmd_vel_pub.move_robot(move_to)
            self.last_move_to = move_to
        elif not detect.success:
            # if there is no collision
            self.cmd_vel_pub.move_robot(self.last_move_to)


if __name__ == "__main__":
    rospy.init_node("Crash_Client_Node")

    client = CrashDetectionClient()

    ctrl_c = False

    def shutdown_hook():
        global ctrl_c

        rospy.logwarn("Shutdown Requested")
        ctrl_c = True

    rospy.on_shutdown(shutdown_hook)
    rate = rospy.Rate(20)

    try:
        while not ctrl_c:
            rospy.loginfo(client.detect())
            client.drive()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
