#!/usr/bin/env python3

from turtle import circle
import numpy as np
import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camcalibration import Calibration as clb
from drone_api import *
from math import pi, sqrt, sin, cos, atan2
import time
import sys


FONT = cv2.FONT_HERSHEY_PLAIN


class Circle:
    def __init__(self):
        self.__buffer = [None] * 25
        self.__yaw = 0
        self.__counter = 0

    def write(self, pose):
        self.__buffer[self.__counter] = pose
        self.__yaw = pose[5]
        self.__counter += 1
        if self.__counter >= len(self.__buffer):
            self.__counter = 0

    def mean(self):
        num = 0
        mean_pose = [0] * 6
        for i in self.__buffer:
            if i is not None:
                num += 1
                for j in range(len(mean_pose)):
                    mean_pose[j] += i[j]
        if num != 0:
            for i in range(len(mean_pose) - 1):
                mean_pose[i] = mean_pose[i] / num
            mean_pose[5] = self.__yaw
        return mean_pose

    def erase_yaw(self):
        self.__yaw = 0


def toFixed(numObj, digits=0):
    return f'{numObj:.{digits}f}'


def callback(data):
    # ROS to OpenCV bridge
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except Exception as e:
        rospy.loginfo(e)
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                          parameters=parameters,
                                                          cameraMatrix=camera_matrix,
                                                          distCoeff=dist_coef)
    global cir
    if np.all(ids is not None):
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners, 0.5, camera_matrix,
                                                                   dist_coef)
        # putting markers info on the screen
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix,
                       dist_coef, rvec[0], tvec[0], 0.2)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, ' id' + str(ids[0])[1:-1], (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
        drone_pose = drone.get_local_pose()
        x, y, z, roll, pitch, yaw = Camera_api.marker_local_pose(rvec[0][0], tvec[0][0],
                                                                 drone_pose, (0, 0, -0.05, 0, pi/2, 0))
        cir.write([x, y, z, roll, pitch, yaw])
        cv2.putText(frame, str(toFixed(x, 3)+'    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 70+20),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(x, 3) + '    ' +
                               toFixed(y, 3) + '    ' +
                               toFixed(z, 3) + '    '), (20, 70+20),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3)+'    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 100+20),
                    FONT, 1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, str(toFixed(roll, 3) + '    ' +
                               toFixed(pitch, 3) + '    ' +
                               toFixed(yaw, 3)), (20, 100+20),
                    FONT, 1, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        cir.erase_yaw()
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame, 'NOT FOUND', (20, 30), FONT,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))
    except Exception as e:
        rospy.loginfo(e)

# loading the coeffs and aruco parameters
camera_matrix, dist_coef = clb.loadCoefficients('calib.yaml')
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

drone = Drone_api()
cir = Circle()
marker_pose = [0, 0, 0, 0, 0, 0]
drone.start()
# Ros start
rospy.loginfo('Drone armed')
image_sub = rospy.Subscriber('/iris_bottom_fpv/usb_cam/image_raw', Image, callback, queue_size=1)
rospy.loginfo('Start Subscriber')
image_pub = rospy.Publisher('/iris_bottom_fpv/usb_cam/location_img', Image, queue_size=1)
rospy.loginfo('Start Publisher')

landing_check = 0
i = 4

drone.set_local_pose(4, 5, 5, 0)
time.sleep(10)
while not drone.is_shutdown():
    marker_pose = cir.mean()
    print(marker_pose)
    drone_pose = drone.get_local_pose()
    if landing_check == 0:
        if marker_pose == [0, 0, 0, 0, 0, 0]:
            print("No markers detected!")
            time.sleep(2)
        else:
            drone.set_local_pose(marker_pose[0], marker_pose[1], marker_pose[2] + 4, 0)
            drone.sleep(0.5)
            if (marker_pose[0] - 0.1 < drone_pose[0] < marker_pose[0] + 0.1) and (marker_pose[1] - 0.1 < drone_pose[1] < marker_pose[1] + 0.1):
                time.sleep(0.5)
                landing_check = 1
                time.sleep(0.5)
    if landing_check == 1:
        while i >= 1:
            marker_pose = cir.mean()
            drone.set_local_pose(marker_pose[0], marker_pose[1], marker_pose[2] + i, 0)
            i -= 1
            time.sleep(1)
        print("Landing successfull.")
        sys.exit()
