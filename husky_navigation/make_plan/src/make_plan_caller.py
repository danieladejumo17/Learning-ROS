#! /usr/bin/env

import rospy
from nav_msgs.srv import GetPlan, GetPlanRequest

rospy.init_node("make_plan")
rospy.loginfo("Node Started")

service = rospy.ServiceProxy("move_base/NavfnROS/make_plan", GetPlan)
service.wait_for_service()

request = GetPlanRequest()
request.start.header.frame_id = "map"
request.start.pose.position.x = 1.16
request.start.pose.position.y = -4.76
request.start.pose.orientation.z = 0.75
request.start.pose.orientation.w = 0.66

request.goal.header.frame_id = "map"
request.goal.pose.position.x = 0.0
request.goal.pose.position.y = -3.50
request.goal.pose.orientation.z = 0.75
request.goal.pose.orientation.w = 0.66

request.tolerance = 0.05
rospy.loginfo("GetPlan Request\n{}".format(request))


response = service(request)
rospy.loginfo("Global Planner Plan:\n{}".format(response))
