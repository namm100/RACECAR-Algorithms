#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from ackermann_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import Joy

from racecar_63.msg import blob_detection

class MasterNode:

	def __init__(self):
		
		self.is_start = False #WHETHER OR NOT THE CAR IS ON OR NOT
		self.is_wall_following = False
		self.following_blobs = True
		self.counter = 0.0
		self.header = std_msgs.msg.Header()
		self.header.stamp = rospy.Time.now()
		
		self.potential_field_sub = rospy.Subscriber("/potential_field", AckermannDriveStamped, self.potential_field_callback)
		self.wall_follower_sub = rospy.Subscriber("/wall_detector" , AckermannDriveStamped, self.wall_follower_callback)
		self.joy_sub = rospy.Subscriber("/vesc/joy", Joy, self.joy_callback)
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
		self.blob_pub = rospy.Subscriber("/target_info", blob_detection, self.blob_callback)
		self.blob_area_threshold = 1750
		
		self.info_text_pub = rospy.Publisher("/info_text", String, queue_size = 10)

	def joy_callback(self, data):
		a_but_pressed = (False,True)[data.buttons[0] == 1]
		b_but_pressed = (False, True)[data.buttons[1] == 1]
		if a_but_pressed:
			self.is_start = True;
		if b_but_pressed:
			self.is_start = False;

	def blob_callback(self, data):
		if not self.following_blobs:
			return
		if self.is_start:
			self.info_text_pub.publish(str(data.size.data))
			if data.size.data > self.blob_area_threshold:
				self.is_wall_following = True	
		
	def potential_field_callback(self, data):
		if self.is_start and not self.is_wall_following:
			rospy.loginfo("potential field")
			self.drive_pub.publish(data)
			self.info_text_pub.publish(String(data="potential_field"))
		
	def wall_follower_callback(self, data):
		if (self.is_start and self.is_wall_following) and self.following_blobs:
			rospy.loginfo("wall following")
			self.info_text_pub.publish(String(data="wall_following"))
			self.counter += 1
			if self.counter == 60:
				self.is_wall_following = False
				self.counter = 0
				self.following_blobs = False
			self.drive_pub.publish(data)
			
if __name__ == "__main__":
	rospy.init_node("grand_prix_node")
	node = MasterNode()
	rospy.spin()
