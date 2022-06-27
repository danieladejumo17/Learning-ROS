#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np

# imu data is published at 50 Hz


class ImuRead():
    def __init__(self, collision_accel_threshold=7.0):
        self.subscriber = rospy.Subscriber(
            "sphero/imu/data3", Imu, self.callback, queue_size=1)
        self.imu = Imu()
        self.collision_accel_threshold = collision_accel_threshold

    def callback(self, imu):
        self.imu = imu

    def get_imu(self):
        return self.imu

    def collision_direction(self):
        imu = self.imu
        accel = np.array([imu.linear_acceleration.x,
                          imu.linear_acceleration.y,
                          imu.linear_acceleration.z])

        if not any(abs(accel) > self.collision_accel_threshold):
            rospy.loginfo("X: {}, Y: {}, Z: {}".format(
                accel[0], accel[1], accel[2]))
            return None

        abs_max_idx = np.argmax(abs(accel))
        max_accel = accel[abs_max_idx]

        message = ""
        positive = max_accel > 0
        if abs_max_idx == 0:
            # Collision is along x-axis
            rospy.logwarn("X-Crash: {}".format(max_accel))
            rospy.loginfo("Y: {}, Z: {}".format(accel[1], accel[2]))

            # It's a side crash???
            if positive:
                message = "right"
            else:
                message = "left"

        elif abs_max_idx == 1:
            # Collision is along the y-axis
            rospy.logwarn("Y-Crash: {}".format(max_accel))
            rospy.loginfo("X: {}, Z: {}".format(accel[0], accel[2]))

            # It's a front/back crash???
            if positive:
                message = "front"
            else:
                message = "back"
        elif abs_max_idx == 2:
            # Collision is along the z-axis. Sphero jumped
            rospy.logwarn("Z-Crash: {}".format(max_accel))
            rospy.loginfo("X: {}, Y: {}".format(accel[0], accel[1]))

            # It's  crash that made it jump
            # If there is such a big z-crash, we assume we've had a
            # big front crash that made sphero jump
            message = "front"
        else:
            rospy.logerr("Invalid accel list index")

        rospy.logwarn("Crash Side: {}".format(message))
        return message


if __name__ == "__main__":
    rospy.init_node("Imu_Reader")
    reader = ImuRead()

    ctrl_c = False

    def shutdown_hook():
        global ctrl_c

        ctrl_c = True
        rospy.loginfo("Shutdown Requested")

    rospy.on_shutdown(shutdown_hook)

    rate = rospy.Rate(1)
    try:
        while not ctrl_c:
            imu = reader.get_imu()
            rospy.loginfo({"Orientation": imu.orientation,
                           "Angular Velocity": imu.angular_velocity,
                           "Linear Acceleration": imu.linear_acceleration})
            rospy.logwarn(reader.collision_direction())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
