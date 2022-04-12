#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

from utils import process_range_data


class WallFollower:
    """ This node drives the robot to follow a wall """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("wall_follower")

        # Get a subscriber to the /scan topic.
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Get a publisher to the /cmd_vel topic.
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # window size for mean-filtering the ranges data
        self.k = 7

        # twist for moving
        self.twist = Twist(linear=Vector3(), angular=Vector3())

        # linear velocity
        self.lin = 0.1

        # desired degree of the closest point
        self.tgt_deg = -90

        # desired distance to the wall
        self.tgt_dist = 0.4

        # angular threshold to enable forward movement
        self.ang_th = 45

        # gains
        self.k_deg = -0.03
        self.k_dist = 1.5

        # keep the node running
        rospy.spin()

    def scan_callback(self, data):
        retval = process_range_data(data, self.k)
        # move forward if nothing in range
        if retval is None:
            self.twist.linear.x = self.lin
            self.twist.angular.z = 0
            self.pub_twist.publish(self.twist)
            return
        deg, dist = retval
        deg_err = self.tgt_deg - deg
        if deg_err < -180:
            deg_err += 360
        # move forward only if angular error is small enough
        self.twist.linear.x = self.lin if abs(deg_err) < self.ang_th else 0
        # apply angular gains
        self.twist.angular.z = deg_err * self.k_deg + (self.tgt_dist - dist) * self.k_dist
        self.pub_twist.publish(self.twist)


if __name__ == '__main__':
    # Declare a node and run it.
    WallFollower()
