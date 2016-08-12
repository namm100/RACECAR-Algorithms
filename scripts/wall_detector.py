#!/usr/bin/python

import rospy
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
from racecar_63.msg import blob_detection

class WallDetectorNode:

    """
    NOTE: A-button starts the sequence, B-button switches walls, and X-button stops the car permanently
    """

    def __init__(self):

        # PID VALUES ---------------------------
        # Wall Tracking Values:
        self.p_val = 1.2
        self.i_val = 0.1
        self.d_val = 0.8
        # --------------------------------------

        self.refresh_count = 0
        self.desired_distance = 0.4
        self.rolling_sum = 0
        self.before_e = 0

        self.is_right = True

        self.drive_pub = rospy.Publisher("/wall_detector", AckermannDriveStamped)

        self.target_vision = rospy.Subscriber("/target_info", blob_detection, self.target_callback)
	self.wall_vision = rospy.Subscriber("/scan", LaserScan, self.wall_callback)

        rospy.init_node("object_detector_node")
        self.header = std_msgs.msg.Header()
        self.header.stamp = rospy.Time.now()

    def target_callback(self, msg):
	    r = msg.color.r
	    g = msg.color.g
	    if r > g:
	        self.is_right = True
	    else:
	        self.is_right = False

    def wall_callback(self, msg):
        width_track = self.find_height(msg.ranges[180], msg.ranges[300], 30) + self.find_height(msg.ranges[780], msg.ranges[900], 30)
	print "width track:", str(width_track)
	self.refresh_count += 1
	if self.is_right:
	    pts = msg.ranges[170:200]
	    pt_one = msg.ranges[180]
	    pt_two = msg.ranges[300]
	else:
	    pts = msg.ranges[880:910]
	    pt_one = msg.ranges[900]
	    pt_two = msg.ranges[780]
	error = self.desired_distance - min(pts)
	d_hat = ((pt_one * pt_two) / (2 * math.sqrt((math.pow(pt_one, 2) + math.pow(pt_two, 2) - math.sqrt(3) * pt_one * pt_two))))
	error_two = self.desired_distance - d_hat
	error_new = (error + error_two) / 2
	if self.refresh_count == 1:
	    self.before_e = error_new
	new_steering_angle = self.pid_controller(error_new)
	drive_command = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=new_steering_angle, speed=3.0))
	self.drive_pub.publish(drive_command)


    def pid_controller(self, error):
        rospy.loginfo(error)
        return self.proportion(error, self.is_right) + self.differential(error, 35.0, self.is_right) + self.integral(error, self.is_right)

    def proportion(self, error, is_right):
        if is_right:
            return float(self.p_val) * float(error)
        else:
            return -1 * float(self.p_val) * float(error)

    def differential(self, error, hz, is_right):
        delta_time = 1 / hz
        edot = float(error - self.before_e) / float(delta_time)
        kd = float(self.d_val)
        self.before_e = error
        if math.fabs(kd) == 0:
            return 0.0
        if is_right:
            return kd * edot
        else:
            return -kd * edot

    def integral(self, error, is_right):
        self.rolling_sum += error
        ki = float(self.i_val)
        if is_right:
            return ki * self.rolling_sum
        else:
            return -1 * ki * self.rolling_sum
            
    def find_height(self, dist_one, dist_two, angle_deg):
        angle_rad = (math.pi * angle_deg) / 180.0
        sin_angle = math.sin(angle_rad)
        cos_angle = math.cos(angle_rad)
        nomin = float(dist_one * dist_two * sin_angle)
        denom = math.sqrt((dist_one ** 2) + (dist_two ** 2) - (2 * dist_one * dist_two * cos_angle))
        return nomin / denom

if __name__ == "__main__":
    node = WallDetectorNode()
    rospy.spin()
