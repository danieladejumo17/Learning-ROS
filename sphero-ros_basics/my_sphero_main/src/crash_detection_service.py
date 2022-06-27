#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from imu_read import ImuRead


# class CrashDetection():
#     def init_node(self, crash_accel_threshold=7):
#         self.imu_reader = ImuRead(crash_accel_threshold)
#         rospy.loginfo("Crash Detection Service Started")
#         self.service = rospy.Service("detect_crash", Trigger, self.detect)

imu_reader = ImuRead(7)


def detect(request):
    global imu_reader

    crash_side = imu_reader.collision_direction()

    response = TriggerResponse()

    if not crash_side:
        response.success = False
        response.message = "forward"    # if there is no collision drive forward
        return response
    else:
        response.success = True     # there is a collision

        # Get the direction too move
        if not crash_side == "front":
            response.message = "forward"
        elif not crash_side == "left":
            response.message = "left"
        elif not crash_side == "right":
            response.message = "right"
        elif not crash_side == "back":
            response.message = "backward"
        else:
            rospy.logerr("Unidentified Crash Side")

        rospy.logwarn("Move-to Direction: {}".format(response.message))
        return response


if __name__ == "__main__":
    rospy.init_node("Crash_Detection_Service", log_level=rospy.INFO)
    rospy.loginfo("Crash Detection Node Started")

    service = rospy.Service("detect_crash", Trigger, detect)
    rospy.loginfo("Detect Crash Service started")
    rospy.spin()
