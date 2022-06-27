#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

# Odom is published at 30 Hz


class OdomRead():
    def __init__(self):
        self.subscriber = rospy.Subscriber(
            "odom", Odometry, self.callback, queue_size=1)
        self.odom = Odometry()

    def callback(self, odom):
        self.odom = odom

    def get_odom(self):
        return self.odom


if __name__ == "__main__":
    rospy.init_node("Odom_Reader")
    reader = OdomRead()

    ctrl_c = False

    def shutdown_hook():
        global ctrl_c

        ctrl_c = True
        rospy.loginfo("Shutdown Requested")

    rospy.on_shutdown(shutdown_hook)

    rate = rospy.Rate(1)
    try:
        while not ctrl_c:
            rospy.loginfo(reader.get_odom().pose.pose.position)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
