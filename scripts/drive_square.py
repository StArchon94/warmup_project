#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist, Vector3


class DriveSquare:
    """ This node drives the robot in a square """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("drive_square")

        # Get a publisher to the cmd_vel topic.
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # forward speed
        lin = 0.3
        # turning speed
        self.ang = 0.5
        # twist for moving forward
        self.fwd_twist = Twist(linear=Vector3(lin, 0, 0), angular=Vector3())
        # twist for turning
        self.turn_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, self.ang))
        # twist for stopping
        self.stop_twist = Twist(linear=Vector3(), angular=Vector3())

    def run(self):
        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        while not rospy.is_shutdown():
            # moving forward for 3 secs
            self.pub_twist.publish(self.fwd_twist)
            rospy.sleep(3)

            # stop moving for 1 sec to stablize
            self.pub_twist.publish(self.stop_twist)
            rospy.sleep(1)

            # turn pi/2
            self.pub_twist.publish(self.turn_twist)
            rospy.sleep(math.pi / 2 / self.ang)

            # stop moving for 1 sec to stablize
            self.pub_twist.publish(self.stop_twist)
            rospy.sleep(1)


if __name__ == '__main__':
    # Declare a node and run it.
    DriveSquare().run()
