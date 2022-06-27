#! /usr/bin/env python

import rospy
from cmd_vel_publisher import CmdVelPub
from odom_read import OdomRead
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from my_sphero_actions.msg import record_odomAction, record_odomGoal


PI = 3.14


class ControlSphero:
    def __init__(self, linear_speed, angular_speed, goal_distance, goal_duration, drive_rate=10):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.goal_distance = goal_distance
        self.goal_duration = goal_duration

        self.init_direction_service()
        self.init_record_odom_action(goal_distance, goal_duration)
        self.init_sphero_vel_publisher(linear_speed, angular_speed)

        self._rate = rospy.Rate(drive_rate)

    def init_direction_service(self):
        self._direction_client = rospy.ServiceProxy("detect_crash", Trigger)
        rospy.loginfo("Waiting for Direction service to start")
        self._direction_client.wait_for_service()
        rospy.loginfo("Direction service available")

        self._direction_service_request = TriggerRequest()

    def init_record_odom_action(self, goal_distance, goal_duration):
        self._record_odom_client = actionlib.SimpleActionClient(
            "record_odom_as", record_odomAction)
        rospy.loginfo("Waiting for Record Odom action server")
        self._record_odom_client.wait_for_server()
        rospy.loginfo("Record Odom action server available")

        self._record_odom_goal = record_odomGoal(duration=goal_duration)

    def init_sphero_vel_publisher(self, linear_speed, angular_speed):
        self._sphero_vel_pub = CmdVelPub(
            linear_speed=linear_speed, angular_speed=angular_speed)

    def get_direction(self):
        return self._direction_client(self._direction_service_request).message

    def send_record_odom_goal(self):
        self._record_odom_client.send_goal(self._record_odom_goal)

    def record_odom_done(self):
        # ENDING=0 ACTIVE=1 PREEMPTED=2 SUCCEEDED=3 ABORTED=4
        # REJECTED=5 PREEMPTING=6 RECALLING=7 RECALLED=8 LOST=9
        return self._record_odom_client.get_state() > 1

    def move_sphero(self, direction):
        self._sphero_vel_pub.move_robot(direction)

    def drive(self):
        rospy.loginfo("Starting the record_odom action")
        self.send_record_odom_goal()

        rospy.loginfo("Starting Sphero drive ...")
        while not self.record_odom_done():
            direction = self.get_direction()
            self.move_sphero(direction)
            self._rate.sleep()

        # if status is SUCCEEDED
        if self._record_odom_client.get_state() == 3:
            record_odom_result = self._record_odom_client.get_result()
            if record_odom_result.exited:
                rospy.loginfo("Sphero out of maze!")
                rospy.loginfo("Sphero used {} seconds".format(
                    record_odom_result.time_used))
            else:
                rospy.loginfo("Sphero in maze. Time up!")
        else:
            rospy.loginfo("An error occured. Action Status: {}".format(
                self._record_odom_client.get_state()))


# get direction from direction service
# move sphero in that direction
# keep doing this while record_odom is not done_robot_ste_robot_steeringering

if __name__ == "__main__":
    rospy.init_node("main", log_level=rospy.INFO)
    rospy.loginfo("Main program started")

    controller = ControlSphero(linear_speed=0.2,
                               angular_speed=0.25,
                               goal_distance=2,
                               goal_duration=120,
                               drive_rate=10)

    rospy.sleep(1)
    controller.drive()
    # rospy.spin()
