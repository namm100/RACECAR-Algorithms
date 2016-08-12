import cv2
import numpy as np

import sys

import rospy
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("challenge_image")

pub_image = rospy.Publisher("output_image", Image, queue_size=1)

def im_cb(image_msg):
    img = BRIDGE.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

    find_blobs(img, None)

rospy.Subscriber("/camera/rgb/image_rect_color", Image, im_cb)
info_pub = rospy.Publisher('/exploring_challenge', String, queue_size=1)

COLORS = [
    # ("green", [60, 60, 40], [70, 255, 255]),
    # ("red", [0, 150, 90], [15, 255, 255]),
    # ("yellow", [20, 150, 90], [30, 255, 255]),
    # ("blue", [110, 150, 90], [130, 255, 255]),
    ("magenta", [145, 50, 80], [170, 255, 255])
]

BRIDGE = CvBridge()

def find_blobs(img, blob_cb):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for label, r1, r2 in COLORS:
        mask = cv2.inRange(hsv, np.array(r1), np.array(r2))

        mask = cv2.GaussianBlur(mask, (21, 21), 0)
        mask = cv2.erode(mask, (5, 5), iterations=5)

        mask_copy = mask.copy()

        contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) < 1:
            continue

        largest = sorted(contours, key= lambda c: cv2.contourArea(c), reverse=True)[0]

        if cv2.contourArea(largest) < 4000:
            continue

        convex = cv2.convexHull(largest)
        approx = cv2.approxPolyDP(convex, 0.05 * cv2.arcLength(convex, closed=True), closed=True)


        if len(approx) != 4:
            continue # not rectangular

        print("AREA", cv2.contourArea(largest))

        M = cv2.moments(approx)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        cv2.drawContours(mask_copy, [approx], -1, (255, 255, 255), 10)

        if blob_cb != None:
            blob_cb(label, (cx, cy))

        # cv2.imshow("thing", cv2.resize(mask_copy, (0,0), fx=0.25, fy=0.25))

        if label == "magenta": # special detection logic
            detect_image(img, approx)

def detect_image(img, approx):
    #warped = four_point_transform(img, approx.reshape((4, 2)))
    warped = four_point_transform(img, approx.reshape((4, 2)))

    pub_image.publish(BRIDGE.cv2_to_imgmsg(warped, "bgr8"))
    #cv2.imshow("Warped", warped)
    #cv2.waitKey(0)
    #label = matcher.match(cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY))
    #print("Found %s" % label)

def order_points(pts):
	# initialzie a list of coordinates that will be ordered
	# such that the first entry in the list is the top-left,
	# the second entry is the top-right, the third is the
	# bottom-right, and the fourth is the bottom-left
	rect = np.zeros((4, 2), dtype = "float32")
 
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
 
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
 
	# return the ordered coordinates
	return rect

INC = 0

def stupid_detector(warped):
    print(warped.shape)
    mx, my = (warped.shape[0] / 2, warped.shape[1] / 2)
    print(mx, my)
    roi = warped[mx-20:mx+20,my-20:my+20,:]
    avg = np.mean(warped, axis=0)
    avg = np.mean(avg, axis=0)

    if abs(avg[0] - 109) < 20 and abs(avg[1] - 96) < 20 and abs(avg[2] - 181) < 20:
        info_pub.publish(String(data="cat"))
    elif abs(avg[0] - 139) < 20 and abs(avg[1] - 111) < 20 and abs(avg[2] - 161) < 20:
        info_pub.publish(String(data="sertac"))
    elif abs(avg[0] - 20) < 20 and abs(avg[1] - 20) < 20 and abs(avg[2] - 20) < 20:
        info_pub.publish(String(data="racecar"))
    else:
        info_pub.publish(String(data="ari"))

    cv2.imwrite("/home/racecar/challenge_photos/photo" + str(INC) + ".png", warped)

    INC += 1

def four_point_transform(image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
 
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
 
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
 
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[100, 0],
		[100, 200],
		[0, 200]], dtype = "float32")
 
	# compute the perspective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (100, 200))
 
	# return the warped image
	return warped


rospy.spin()
