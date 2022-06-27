#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


goals = [[2.0, 2.0],
         [-2.0, -2.0],
         [-2.0, 2.0]]

rospy.init_node("send_goals")

client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server()

while not rospy.is_shutdown():
    for goal in goals:
        goal_message = MoveBaseGoal()
        goal_message.target_pose.pose.position.x = goal[0]
        goal_message.target_pose.pose.position.y = goal[1]
        goal_message.target_pose.pose.position.z = 0.0
        goal_message.target_pose.pose.orientation.z = 0.999905477309
        goal_message.target_pose.pose.orientation.w = -0.013749052577

        goal_message.target_pose.header.frame_id = "map"

        client.send_goal(goal_message)
        client.wait_for_result()
        status = client.get_state()
        if status < 2:
            rospy.loginfo("Error Processing Goal. Status {}".format(status))
        rospy.loginfo("Completed goal {}".format(goal))
        # print(f`Completed goal {goal}`)
