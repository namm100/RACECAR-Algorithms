#!/usr/bin/env python
import numpy as np
import sys
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from racecar_63.msg import blob_detection
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Header, ColorRGBA
import RacecarUtilities


class blob_detector:

    def __init__(self):
        def nothing(x):
            pass

        if len(sys.argv) == 2:
            self.image = np.zeros((720, 1280, 3), np.uint8)
            self.finalMask = self.image
            cv2.namedWindow('HSV')
            cv2.createTrackbar('HL', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VL', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('HU', 'HSV', 0, 180, nothing)
            cv2.createTrackbar('SU', 'HSV', 0, 255, nothing)
            cv2.createTrackbar('VU', 'HSV', 0, 255, nothing)
            self.hl = 0
            self.sl = 0
            self.vl = 0
            self.hu = 0
            self.su = 0
            self.vu = 0
            self.is_tuning = sys.argv[1]
            self.window_thread = RacecarUtilities.StoppableThread(target=self.window_runner)
            self.window_thread.start()
        else:
            self.is_tuning = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.image_callback)
        self.info_pub = rospy.Publisher('target_info', blob_detection, queue_size=1)

    def image_callback(self, image_msg):
        img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        imageHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if self.is_tuning:
            TESTING_BOUNDS = [np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])]
            self.finalMask = cv2.inRange(imageHSV, TESTING_BOUNDS[0], TESTING_BOUNDS[1])
        else:
            GREEN_BOUNDS = [np.array([40, 110, 80]), np.array([80, 255, 255])]  # Green filter values
            RED_BOUNDS = [np.array([0, 180, 80]), np.array([16, 255, 255])]  # Red filter values
            redMask = cv2.inRange(imageHSV, RED_BOUNDS[0], RED_BOUNDS[1])
            greenMask = cv2.inRange(imageHSV, GREEN_BOUNDS[0], GREEN_BOUNDS[1])
            self.finalMask = cv2.add(redMask, greenMask)
        self.finalMask = cv2.erode(self.finalMask, (5,5), iterations=5)
        filteredHSV = cv2.bitwise_and(imageHSV, imageHSV, mask=self.finalMask)
        self.image = cv2.cvtColor(filteredHSV, cv2.COLOR_HSV2BGR)
        # grayscale = cv2.cvtColor(imageBGR, cv2.COLOR_RGB2GRAY)
        contours, h = cv2.findContours(self.finalMask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        # for drawing the contours themselves, uncomment the following line:
        # cv2.drawContours(rgb_new, contours, -1, (255,0,0), thickness=3)
        # rospy.loginfo(len(contours))
        contours = sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            rect_area = w * h
            extent = float(area) / rect_area
            if extent > 0 and self.find_area(contour) > 2000:
                rect = cv2.minAreaRect(contour)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                msg_header = Header()
                msg_header.stamp = rospy.Time.now()
                c = self.find_center(contour)
                pos = Point(x=c[0], y=c[1], z=0.0)
                blob_message = blob_detection(header=msg_header,
                                              color=ColorRGBA(r=self.image[c[1], c[0], 2], g=self.image[c[1], c[0], 1] , b=self.image[c[1], c[0], 0], a=1.0),
                                              size=Float64(data=self.find_area(contour)),
                                              location=pos)
                self.info_pub.publish(blob_message)
                if self.is_tuning:
                    cv2.drawContours(self.image, [box], 0, (255, 0, 0), 5)
                    cv2.circle(self.image, self.find_center(contour), 8, (255, 0, 0), thickness=5)
                break

    def find_area(self, contour):
        moments = cv2.moments(contour)

        return moments['m00']

    def find_center(self, contour):
        moments = cv2.moments(contour)
        x_val = int(moments['m10'] / moments['m00'])
        y_val = int(moments['m01'] / moments['m00'])

        return x_val, y_val

    def window_runner(self):
        cv2.imshow('HSV', cv2.resize(self.image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            self.window_thread.stop()

        self.hl = cv2.getTrackbarPos('HL', 'HSV')
        self.sl = cv2.getTrackbarPos('SL', 'HSV')
        self.vl = cv2.getTrackbarPos('VL', 'HSV')
        self.hu = cv2.getTrackbarPos('HU', 'HSV')
        self.su = cv2.getTrackbarPos('SU', 'HSV')
        self.vu = cv2.getTrackbarPos('VU', 'HSV')


if __name__ == "__main__":
    rospy.init_node('blob_detection_node')
    node = blob_detector()
    rospy.spin()
    cv2.destroyAllWindows()
