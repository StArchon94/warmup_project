#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

from utils import process_range_data


class PersonFollower:
    """ This node drives the robot to follow a person """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("person_follower")

        # Get a subscriber to the /scan topic.
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Get a publisher to the /cmd_vel topic.
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # window size for mean-filtering the ranges data
        self.k = 7

        # twist for moving
        self.twist = Twist(linear=Vector3(), angular=Vector3())

        # twist for stopping
        self.stop_twist = Twist(linear=Vector3(), angular=Vector3())

        # desired distance to the person
        self.tgt_dist = 0.5

        # angular threshold to enable forward movement
        self.ang_th = 45

        # gains
        self.k_dist = -0.5
        self.k_deg = 0.02

        # keep the node running
        rospy.spin()

    def scan_callback(self, data):
        retval = process_range_data(data, self.k)
        # stop if nothing in range
        if retval is None:
            self.pub_twist.publish(self.stop_twist)
            return
        deg, dist = retval
        # apply linear gain only if angular error is small enough
        self.twist.linear.x = (self.tgt_dist - dist) * self.k_dist if abs(deg) < self.ang_th else 0
        # apply angular gain
        self.twist.angular.z = deg * self.k_deg
        self.pub_twist.publish(self.twist)


if __name__ == '__main__':
    # Declare a node and run it.
    PersonFollower()
