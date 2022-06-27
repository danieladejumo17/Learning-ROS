#! /usr/bin/env python

import rospy
import actionlib
from my_sphero_actions.msg import record_odomAction, record_odomGoal


# simple state will store 1 when active and 2 when finished
# get_state returns a more deatailed status information: 1 to 9
# ENDING=0 ACTIVE=1 PREEMPTED=2 SUCCEEDED=3 ABORTED=4 REJECTED=5 PREEMPTING=6 RECALLING=7 RECALLED=8 LOST=9

class RecordOdomClient:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient(
            "record_odom_as", record_odomAction)
        self.action_client.wait_for_server()
        rospy.loginfo("record_odom_as available")

    def start_recording(self, duration):
        self.action_client.send_goal(record_odomGoal(duration=duration),
                                     done_cb=self.handle_result,
                                     feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        rospy.loginfo("Time Left: {}".format(feedback.time_left))
        rospy.loginfo("Simple State: {}".format(
            self.action_client.simple_state))
        rospy.loginfo("State: {}".format(self.action_client.get_state()))

    def handle_result(self, status, result):
        rospy.loginfo("Result: {}".format(result))
        rospy.loginfo("Status: {}".format(status))
        rospy.loginfo("Simple State: {}".format(
            self.action_client.simple_state))
        rospy.loginfo("State: {}".format(self.action_client.get_state()))


if __name__ == "__main__":
    rospy.init_node("record_odom_client")
    rospy.loginfo("record_odom_client started")

    record_odom = RecordOdomClient()
    record_odom.start_recording(10)

    rospy.spin()
