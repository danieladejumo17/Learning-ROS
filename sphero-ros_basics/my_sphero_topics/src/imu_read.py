#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np


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
            return None

        abs_max_idx = np.argmax(abs(accel))
        max_accel = accel[abs_max_idx]

        if abs_max_idx == 0:
            # Collision is along x-axis
            return {"direction": "X", "value": max_accel}
        elif abs_max_idx == 1:
            # Collision is along the y-axis
            return {"direction": "Y", "value": max_accel}
        elif abs_max_idx == 2:
            # Collision is along the z-axis. Sphero is jumping
            return {"direction": "Z", "value": max_accel}


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
