#! /usr/bin/env python

import rospy
from get_pose.srv import GetPose, GetPoseResponse
from geometry_msgs.msg import PoseWithCovarianceStamped


class GetPoseClass:
    def __init__(self, topic="amcl_pose"):
        self.topic = topic
        self.service = rospy.Service("get_robot_pose", GetPose, self.pose_data)

        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                         self.update_pose, queue_size=1)
        self.currentPose = None

    def pose_data(self, _):
        rospy.loginfo("Current Pose: {}".format(self.currentPose))
        return GetPoseResponse(self.currentPose)

    def update_pose(self, pose):
        self.currentPose = pose.pose.pose


if __name__ == "__main__":
    rospy.init_node("get_pose")
    GetPoseClass()
    rospy.spin()
