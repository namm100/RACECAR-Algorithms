#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
import math

class Acceleration():
    prevSpeed = 0
    deltaSpeed = 0.1
    maxSpeed = 2

    def __init__(self):
        rospy.init_node("trap_acceleration")

        rospy.Subscriber("potential_field", AckermannDriveStamped, callback)

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

    def callback(data):
        if(sign(data.drive.speed * prevSpeed) == 1):
            data.drive.speed = min(prevSpeed + deltaSpeed * sign(data.drive.speed), maxSpeed)
        else:
            data.drive.speed = 0

        prevSpeed = data.drive.speed
        self.drive_pub.publish(data)

node = Acceleration()

rospy.spin()
