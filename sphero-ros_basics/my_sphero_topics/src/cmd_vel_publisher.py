#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class CmdVelPub():
    def __init__(self, linear_speed=0.2, angular_speed=0.5):
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.twist = Twist()

    def move_robot(self, direction):
        if direction == "forward":
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = 0
        elif direction == "backward":
            self.twist.linear.x = -self.linear_speed
            self.twist.angular.z = 0
        elif direction == "right":
            self.twist.linear.x = 0
            self.twist.angular.z = -self.angular_speed
        elif direction == "left":
            self.twist.linear.x = 0
            self.twist.angular.z = self.angular_speed
        elif direction == "stop":
            self.twist.linear.x = 0
            self.twist.angular.z = 0

        self.cmd_vel_pub.publish(self.twist)


if __name__ == "__main__":
    rospy.init_node("CmdVelPublisher")

    cmd_vel_pub = CmdVelPub()
    rate = rospy.Rate(1)
    ctrl_c = False

    def on_shutdown():
        global ctrl_c
        rospy.loginfo("Shutting Down CmdVelPub!")

        ctrl_c = True
        cmd_vel_pub.move_robot("stop")

    rospy.on_shutdown(on_shutdown)

    try:
        while not ctrl_c:
            rospy.loginfo("Moving Bot")
            cmd_vel_pub.move_robot("forward")
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
