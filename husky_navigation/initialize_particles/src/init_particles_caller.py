#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty

rospy.init_node("initialize_particles")

serviceProxy = rospy.ServiceProxy("global_localization", Empty)
serviceProxy.wait_for_service()

serviceProxy()
rospy.loginfo("Global Localization Service Called")
