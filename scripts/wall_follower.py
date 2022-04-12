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

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Get a publisher to the cmd_vel topic.
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # window size for mean-filtering the ranges data
        self.k = 7

        # twist for moving
        self.twist = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3())

        # target distance to maintain
        self.tgt_deg = -90
        self.tgt_dist = 0.4

        # gains
        self.k_deg = -0.03
        self.k_dist = 1.5

        # keep the node running
        rospy.spin()

    def scan_callback(self, data):
        retval = process_range_data(data, self.k)
        if retval is None:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
            self.pub_twist.publish(self.twist)
            return
        deg, dist = retval
        deg_diff = self.tgt_deg - deg
        if deg_diff < -180:
            deg_diff += 360
        self.twist.linear.x = 0.1 if abs(deg_diff) < 45 else 0
        self.twist.angular.z = deg_diff * self.k_deg + (self.tgt_dist - dist) * self.k_dist
        self.pub_twist.publish(self.twist)


if __name__ == '__main__':
    # Declare a node and run it.
    WallFollower()
