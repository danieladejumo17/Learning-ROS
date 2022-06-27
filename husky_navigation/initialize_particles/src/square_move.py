#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
import numpy as np


class MoveSquare():
    PI = 3.14

    def __init__(self, length):
        self.square_length = length

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.amcl_pose_sub = rospy.Subscriber(
            "amcl_pose", PoseWithCovarianceStamped, self.update_cov)
        self.twist = Twist()
        self.amcl_pose_cov = None

        self.serviceProxy = rospy.ServiceProxy("global_localization", Empty)
        self.serviceProxy.wait_for_service()

    def update_cov(self, pwc):
        cov = pwc.pose.covariance
        self.amcl_pose_cov = [cov[0], cov[8], cov[-1]]

    def check_amcl_cov(self, thresh):
        rospy.loginfo("Covariance: {}".format(self.amcl_pose_cov))
        if not self.amcl_pose_cov:
            return False
        return all(np.array(self.amcl_pose_cov) < thresh)

    def publishTwist(self, duration):
        rate = rospy.Rate(100)
        milli = duration * 100
        for _ in range(int(milli)):
            self.pub.publish(self.twist)
            rate.sleep()

    def turn90(self, turn_speed=-0.6):
        self.twist.angular.z = turn_speed  # rad/s
        # self.twist.linear.x = 0
        self.publishTwist((self.PI/2)/abs(turn_speed))

    def moveSquareSide(self, move_speed=0.2):
        self.twist.angular.z = 0
        self.twist.linear.x = move_speed  # m/s
        self.publishTwist(self.square_length/abs(move_speed))

    def move(self):
        for i in range(4):
            rospy.loginfo("Moving Side {}".format(i))
            self.moveSquareSide()
            rospy.loginfo("Turning Side {}".format(i))
            self.turn90()
        self.twist = Twist()
        self.publishTwist(1)

    def localize(self):
        while not self.check_amcl_cov(0.65):
            self.serviceProxy()
            self.move()
            rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node("move_square")
    ms = MoveSquare(1)
    rospy.loginfo("Moving Husky in a Square")
    ms.localize()
    rospy.loginfo("Move Square and Localization Completed")
