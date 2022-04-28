#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap

rospy.init_node("get_map_data")
rospy.loginfo("Node Initialized")
service = rospy.ServiceProxy("static_map", GetMap)
service.wait_for_service()
rospy.loginfo("Service is ready")

response = service()
rospy.loginfo("Resolution {}, Dimension: {} x {}".format(
    response.map.info.resolution, response.map.info.width, response.map.info.height))
