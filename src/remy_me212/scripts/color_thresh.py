#!/usr/bin/env python

# 2.12 Lab 7 object detection: a node for color thresholding
# Jacob Guggenheim 2019
# Jerry Ng 2019

import rospy
import numpy as np
import cv2  # OpenCV module
import tf, tf2_ros
from remy_me212.msg.Topping import *
from remy_me212.msg import Topping, ToppingArray
from std_msgs.msg import Bool

#print(cv2.__version__)
import imutils
from matplotlib import pyplot as plt
import time
# from Tkinter import *
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

rospy.init_node('colorThresh', anonymous=False)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()
delta_out_of_the_way = False
# y = 234
# x = -5.5
# z = -30
# 165 degrees rotated


def send_camera_transform():
    angle = 165 * np.pi / 180
    b = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (234, 5.5, -30)
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = tf.transformations.quaternion_from_euler(0, angle, 0)
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "usb_cam"
    b.sendTransform(t)

def delta_out_of_the_way_callback(data):
    global delta_out_of_the_way
    delta_out_of_the_way = data.data


def main():
    rospy.init_node('colorThresh', anonymous=False)
    send_camera_transform()
    rospy.Subscriber('delta_out_of_the_way', Bool, delta_out_of_the_way_callback)
    while not delta_out_of_the_way:
        rospy.sleep(.1)
    rospy.Subscriber('/usb_cam/image_raw', Image, colorThreshCallback)
    send_camera_transform()
    rospy.spin()


def colorThreshCallback(msg):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv_image[50:580, 400:870]
    except CvBridgeError as e:
        print(e)

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #               ['name',    [hsv lower].    [hsv upper],        [thr. val], True/False for contour approx ]
    shape_params = [['black',   [0, 0, 0],      [184, 26, 36],      [0, 255],   True,  100],
                    ['red',     [154, 30, 0],   [191, 109, 101],    [0, 255],   False, 200],
                    ['yellow',  [23, 38, 98],   [53, 188, 158],     [35, 255],  False, 100],
                    ['pink',    [139, 23, 98],  [221, 255, 255],    [35, 255],  False, 100],
                    ['blue',    [105, 101, 45], [150, 255, 255],    [35, 255],  False, 40],
                    ['tan',     [0, 45, 0],     [100, 150, 124],    [0, 225],   True,  40]]
    for shape in shape_params:
        lower_bound_HSV = np.array(shape[1])
        upper_bound_HSV = np.array(shape[2])
        mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)
        disp_image_HSV = cv2.bitwise_and(cv_image, cv_image, mask=mask_HSV)
        image = disp_image_HSV
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # thresh = cv2.threshold(blurred, 35, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.threshold(blurred, shape[3][0], shape[3][1], cv2.THRESH_BINARY)[1]
        if shape[0] == 'tan':
            erosion = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            circles = cv2.HoughCircles(erosion, cv2.HOUGH_GRADIENT, 1, erosion.shape[0] / 25,
                                       param1=100, param2=9,
                                       minRadius=17, maxRadius=27)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])

                    cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
                    radius = i[2]
                    cv2.circle(cv_image, center, radius, (255, 0, 255), 3)  # dictionary of contours

        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            if shape[4] == True:
                epsilon = 0.08 * cv2.arcLength(c, True)
                c = cv2.approxPolyDP(c, epsilon, True)
            if (
                    (round(cv2.contourArea(c)) > shape[5]
                     and cv2.contourArea(c) < 500
                     and 30 < cv2.arcLength(c, True) < 120
                     and shape[0] != 'tan')
                    or (shape[0] == 'tan'
                        and 1000 < cv2.contourArea(c)
                        and 300 < cv2.arcLength(c, True))
            ):
                # compute the center of the contour
                # print("area",cv2.contourArea(c))
                # print("arc length", cv2.arcLength(c, True))
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # draw the contour and center of the shape on the image
                cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, shape[0], (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #TODO: K means clustering to get best centroids for each topping and the pizza. Probably run the subscriber for 2
    # seconds or so, get a set of means for each category of thing, do k-means, and then publish to the planner, along
    # with the stage 0 complete flag
    #TODO: Also, I need to use the right image_geometry function and some transforms to get the actual location of each
    # topping before I publish it.

    # show the image
    cv2.imshow("Image", cv_image)
    cv2.waitKey(3)

    #############CIRCLE DETECTION######################
    # # gray = cv2.threshold(blurred, l_b.get(), u_b.get(), cv2.THRESH_BINARY)[1]
    # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # gray = cv2.medianBlur(gray, 5)
    # rows = gray.shape[0]
    # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
    #                           param1=100, param2=30,
    #                           minRadius=1, maxRadius=30)
    #
    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0, :]:
    #         center = (i[0], i[1])
    #         # circle center
    #         cv2.circle(cv_image, center, 1, (0, 100, 100), 3)
    #         # circle outline
    #         radius = i[2]
    #         cv2.circle(cv_image, center, radius, (255, 0, 255), 3) #dictionary of contours with locations and areas of each circle
    #         #if two circles are close in coordionates (less than small area of olive) than delete smaller one and
    #
    # cv2.imshow("detected circles", cv_image)
    # cv2.waitKey(3)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptionException:
        pass
