#!/usr/bin/env python
import rospy
import tf
import math

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String

PUSH_MULTIPLIER = rospy.get_param("/push_multiplier")
rospy.loginfo("push mul %f", PUSH_MULTIPLIER)
STEER_GRAD_PROPORTION = rospy.get_param("/steer_grad_proportion", default=900.0)
SPEED_GRAD_PROPORTION = rospy.get_param("/speed_grad_proportion", default=-0.001)
MOMENTUM_MU = rospy.get_param("/momentum_mu", default=0.90)
UPDATE_INFLUENCE = rospy.get_param("/update_influence", default=0.08)
MIN_SPEED_CLAMP = rospy.get_param("/min_speed_clamp", default=-1)
MAX_SPEED_CLAMP = rospy.get_param("/max_speed_clamp", default=2)
REVERSE_SPEED_MULTIPLIER = rospy.get_param("/reverse_speed_multiplier", default=-2.3)
STEER_BIAS = rospy.get_param("/steer_bias", default=0)
print(PUSH_MULTIPLIER)

# Get the gradient of the potential of an obstacle
# particle at (ox, oy) with the origin at (mx, my)
def grad(dist, mx, my, ox, oy):
    c = -1/((mx - ox)**2 + (my - oy)**2)**1.5
    return c*(mx - ox), c*(my - oy)

# Get the potential of an obstacle particle at (ox, oy)
# with the origin at (mx, my)
def potential(mx, my, ox, oy):
    1.0 / ((mx - ox)**2 + (my - oy)**2)**0.5

class PotentialFieldNode():
    def __init__(self):
        rospy.init_node("potential_field_node")

        # Lidar subscriber
        rospy.Subscriber("/scan", LaserScan, self.receive_lidar)

        # Driving publisher
        self.drive_pub = rospy.Publisher("/potential_field", AckermannDriveStamped, queue_size=10)

        # cumulative speed - used to build up momentum
        self.speed_c = 0
        
        #info publisher
        self.info_pub = rospy.Publisher("info_text", String)


    # calculate the total gradient from an array of lidar ranges
    # with origin at (my_x, my_y)
    def calc_gradient(self, ranges, my_x, my_y):
        gradient_x = 0 # sum of dU/dx
        gradient_y = 0 # sum of dU/dy

        # ignore the edges of the lidar FOV, usually noisy
        for i in range(180, len(ranges) - 180):
            r = ranges[i]
            deg = (270.0/1080) * i # convert index of range to degree of range
            deg -= 45 # lidar FOV starts at -45 deg
            px = r * math.cos(math.radians(deg)) # convert from polar to x coord
            py = r * math.sin(math.radians(deg)) # convert from polar to y coord
            gx, gy = grad(r, my_x, my_y, px, py) # compute gradient at rectangular coordinates

            # add point's gradient into sum
            gradient_x += gx
            gradient_y += gy

        return (gradient_x, gradient_y)

    # lidar subscriber callback
    def receive_lidar(self, scan_msg):
        # compute gradient sums from lidar ranges
        grad_x, grad_y = self.calc_gradient(scan_msg.ranges, 0, 0)

        grad_x += STEER_BIAS * grad(0.1, 0, 0, 0.1, 0)[0]
        # place repelling particle behind origin (the car) to
        # push the car forward. 14 is a multiplier to give more push.
        grad_y += PUSH_MULTIPLIER * grad(0.1, 0, 0, 0, -0.1)[1]

        #print(grad_x, grad_y)

        # magnitude of gradient (euclidian dist)
        grad_magnitude = math.sqrt(grad_x**2 + grad_y**2)

        # steering proportional to potential gradient w.r.t. x
        steer = grad_x / STEER_GRAD_PROPORTION # OR? math.atan2(grad_x, grad_y)

        # the speed update at this instance: proportional to gradient magnitude
        # and sign depends of sign of gradient w.r.t y
        speed = SPEED_GRAD_PROPORTION * grad_magnitude * np.sign(grad_y)

        # update the cumulative momentum using the speed update at this instance.
        # speed_c is multiplied by some constant < 1 to simulate friction and
        # speed is multiplied by some constant > 0 to determine the influence of the
        # speed update at this instance.
        self.speed_c = MOMENTUM_MU*self.speed_c + UPDATE_INFLUENCE * speed
        
        

        # if speed is less than -1, clamp it. also, the steering is multiplied
        # by a negative constant < -1 to make it back out in a way that
        # orients the car in the direction it would want to turn if it were
        # not too close.
        speed_now = self.speed_c
        if self.speed_c < 0:
            if self.speed_c > -0.2:
                speed_now = -0.7
            steer *= REVERSE_SPEED_MULTIPLIER
            print("reversing")

        if self.speed_c < MIN_SPEED_CLAMP:
            speed_now = MIN_SPEED_CLAMP 
        elif self.speed_c > MAX_SPEED_CLAMP:
            # if speed is greater than 1, clamp it
            speed_now = MAX_SPEED_CLAMP
        self.info_pub.publish(String(data="Speed: " + str(speed_now)))

        # create and publish drive message using steer and speed_c
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = speed_now

        self.drive_pub.publish(drive_msg)

node = PotentialFieldNode()

rospy.spin()
