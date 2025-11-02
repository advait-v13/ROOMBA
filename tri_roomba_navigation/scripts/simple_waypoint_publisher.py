#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('simple_waypoint_publisher')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    seq = [
        (0.2, 0.0, 0.0),
        (0.0, 0.2, 0.0),
        (-0.2, 0.0, 0.0),
        (0.0, -0.2, 0.0),
        (0.0, 0.0, 0.5)
    ]
    i = 0
    while not rospy.is_shutdown():
        tx, ty, tz = seq[i % len(seq)]
        t = Twist()
        t.linear.x = tx
        t.linear.y = ty
        t.angular.z = tz
        pub.publish(t)
        i += 1
        rate.sleep()
