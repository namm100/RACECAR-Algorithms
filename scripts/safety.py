import rospy 
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node("safety_stop")

drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)

def scan_cb(msg):
	dist = msg.ranges[len(msg.ranges) / 2]
	if dist < 0.3:
		msg = AckermannDriveStamped()
		msg.drive.speed = 0
		drive_pub.publish(msg)

rospy.Subscriber("/scan", LaserScan, scan_cb)

rospy.spin()
