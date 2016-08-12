#!/usr/bin/env python
import numpy as np
import sys
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import RacecarUtilities


class blob_detector:

    def __init__(self):
        def nothing(x):
            pass

        if len(sys.argv) == 2:
            self.image = np.zeros((720, 1280, 3), np.uint8)
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
        self.info_pub = rospy.Publisher('/exploring_challenge', String, queue_size=1)

    def image_callback(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.is_tuning:
            TESTING_BOUNDS = [np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])]
            finalMask = cv2.inRange(imageHSV, TESTING_BOUNDS[0], TESTING_BOUNDS[1])
            self.find_contour(finalMask, imageHSV)
        else:
            GREEN_BOUNDS = [np.array([40, 110, 80]), np.array([80, 255, 255])]  # Green filter values
            RED_BOUNDS = [np.array([0, 180, 90]), np.array([15, 255, 255])]  # Red filter values
            YELLOW_BOUNDS = [np.array([26, 86, 94]), np.array([40, 255, 255])]  # Yellow filter values
            BLUE_BOUNDS = [np.array([100, 125, 40]), np.array([130, 255, 80])]  # Blue filter values
            redMask = cv2.inRange(imageHSV, RED_BOUNDS[0], RED_BOUNDS[1])
            greenMask = cv2.inRange(imageHSV, GREEN_BOUNDS[0], GREEN_BOUNDS[1])
            yellowMask = cv2.inRange(imageHSV, YELLOW_BOUNDS[0], YELLOW_BOUNDS[1])
            blueMask = cv2.inRange(imageHSV, BLUE_BOUNDS[0], BLUE_BOUNDS[1])
            self.find_contour(redMask, imageHSV)
            self.find_contour(greenMask, imageHSV)
            self.find_contour(yellowMask, imageHSV)
            self.find_contour(blueMask, imageHSV)

    def find_contour(self, img_mask, image):
        new_mask = cv2.erode(img_mask, (5,5))
        filtered = cv2.cvtColor(cv2.bitwise_and(image, image, mask=new_mask), cv2.COLOR_HSV2BGR)
        self.image = filtered
        contours, h = cv2.findContours(img_mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            hull = cv2.convexHull(contour)
            if cv2.contourArea(contour) > 2500:
                rect = cv2.minAreaRect(contour)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                c = self.find_center(contour)
                h = image[c[1], c[0], 0]
                epsilon = cv2.arcLength(hull, True)*0.02
                print epsilon
                approxCurve = cv2.approxPolyDP(contour, epsilon, True)
                numSides = len(approxCurve)
                rospy.loginfo("Sides: "+str(numSides))
                msg = String(data="None")
                if 40 < h < 80:
                    msg.data = "GREEN"
                elif 0 < h < 15:
                    msg.data = "RED"
                elif 100 < h < 130:
                    msg.data = "BLUE"
                elif 26 < h < 40:
                    msg.data = "YELLOW"
                if numSides == 3:
                    msg.data += " TRIANGLE"
                elif numSides == 4:
                    msg.data += " RECTANGLE"
                elif numSides == 12:
                    msg.data += " CROSS"
                else:
                    msg.data += " CIRCLE"
                cv2.drawContours(self.image, [box], 0, (255, 0, 0), 5)
                cv2.circle(self.image, c, 8, (255, 0, 0), thickness=5)
                img_name = msg.data.replace(" ", "")
                cv2.imwrite("/home/racecar/challenge_photos/" + img_name + ".png", self.image)
                self.info_pub.publish(msg)
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
