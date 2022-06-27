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

    collision_direction = imu_reader.collision_direction()

    response = TriggerResponse()

    if not collision_direction:
        response.success = False
        response.message = "nothing"
        return response
    else:
        response.success = True     # there is a collision

        positive = collision_direction["value"] > 0

        rospy.loginfo(collision_direction)
        if collision_direction["direction"] == 'X':
            if positive:
                response.message = "forward"
            else:
                response.message = "backward"
        elif collision_direction["direction"] == 'Y':
            if positive:
                response.message = "right"
            else:
                response.message = "left"
        else:   # we are not sure of what direction to turn
            response.message = "nothing"

        return response


if __name__ == "__main__":
    rospy.init_node("Crash_Detection_Service")
    rospy.loginfo("Crash Detection Node Started")

    service = rospy.Service("detect_crash", Trigger, detect)
    rospy.loginfo("Detect Crash Service started")
    rospy.spin()
