#! /usr/bin/env python

import rospy
import actionlib
from my_sphero_actions.msg import record_odomAction, record_odomFeedback, record_odomResult
from odom_read import OdomRead

import numpy as np


class RecordOdom:
    def __init__(self, record_rate=1):
        self.action_server = actionlib.SimpleActionServer(
            "record_odom_as", record_odomAction, execute_cb=self.handle_goal, auto_start=False)

        self.odom_array = []

        self.odom_reader = OdomRead()
        self.rate = rospy.Rate(record_rate)

        self.dist_thresh = 4
        self.angle_thresh = [-30, 30]

        self.action_server.start()

    def handle_goal(self, goal):
        self.odom_array = []

        start_time = rospy.Time.now()

        def duration():
            now = rospy.Time.now()
            return (now - start_time).to_sec()

        index = 0
        success = True
        while duration() < goal.duration:
            if self.action_server.is_preempt_requested():
                rospy.logwarn("Preempt Requested.")
                self.action_server.set_preempted()
                success = False
                break

            if self.out_of_maze():
                rospy.loginfo("Sphero out of maze!")
                break

            rospy.loginfo("Recording Odom Index {}".format(index))
            self.odom_array.append(self.odom_reader.get_odom())
            index += 1

            feedback = record_odomFeedback()
            feedback.time_left = goal.duration - duration()
            self.action_server.publish_feedback(feedback)

            self.rate.sleep()

        if success:
            time_used = duration()

            result = record_odomResult()
            result.exited = self.out_of_maze()
            result.result_odom_array = self.odom_array
            result.time_used = time_used

            self.action_server.set_succeeded(result)

    def out_of_maze(self):
        if len(self.odom_array) == 0:
            return False

        start_position = self.odom_array[0].pose.pose.position
        end_position = self.odom_array[-1].pose.pose.position

        dist_x = end_position.x - start_position.x
        dist_y = end_position.y - start_position.y

        dist_vec = np.array([dist_x, dist_y])
        dist = np.linalg.norm(dist_vec)

        return dist > self.dist_thresh


if __name__ == "__main__":
    rospy.init_node("record_odom_server")
    rospy.loginfo("record_odom_server node started")

    record_odom = RecordOdom()
    rospy.loginfo("record_odom_as action server started")

    rospy.spin()
