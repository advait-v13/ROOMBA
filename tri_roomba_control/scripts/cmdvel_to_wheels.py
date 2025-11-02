#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToWheels:
    def __init__(self):
        rospy.init_node('cmdvel_to_wheels')
        self.r = rospy.get_param('~wheel_radius', 0.05)
        self.L = rospy.get_param('~wheel_base', 0.17)  # distance from center to wheel
        self.phi = [0.0, 2.09439510239319549, -2.09439510239319549]

        self.pub_front = rospy.Publisher('/wheel_front_velocity_controller/command', Float64, queue_size=1)
        self.pub_left = rospy.Publisher('/wheel_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_right = rospy.Publisher('/wheel_right_velocity_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        rospy.loginfo("cmdvel_to_wheels ready")
        rospy.spin()

    def cmd_cb(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        wheel_vels = []
        for phi in self.phi:
            wi = ( -math.sin(phi) * vx + math.cos(phi) * vy + self.L * wz ) / self.r
            wheel_vels.append(wi)
        self.pub_front.publish(Float64(wheel_vels[0]))
        self.pub_left.publish(Float64(wheel_vels[1]))
        self.pub_right.publish(Float64(wheel_vels[2]))

if __name__ == '__main__':
    try:
        CmdVelToWheels()
    except rospy.ROSInterruptException:
        pass
