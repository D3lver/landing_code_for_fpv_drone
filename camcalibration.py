#!/usr/bin/env python3

# =========================
#  calibration
# =========================
# https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844

import numpy as np
import time
import cv2
import cv2.aruco as aruco

FONT = cv2.FONT_HERSHEY_PLAIN


class Calibration:
    def __init__(self, square_size=0.0225, width=9, height=6):
        # create the tablet
        self.__square_size = square_size  # size of the ssquare in meters
        self.__width = width    # number of Horisontal squares
        self.__height = height  # number of vertical squares
        self.__images = []

        # cams characteristics
        self.__camera_matrix = None
        self.__dist_coef = None

    def setImages(self, period, num_of_images):
        cap = cv2.VideoCapture(0)
        last_time = time.time()
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        # def callback(data):
        while len(self.__images) < num_of_images:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            now = time.time()
            passed = now - last_time
            if passed < period:
                if passed < 0.25:
                    frame = cv2.bitwise_not(frame)
                cv2.putText(frame, str(passed), (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 0, 60), 2, cv2.LINE_AA)
                cv2.putText(frame, str(len(self.__images)), (20, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 0, 60), 2, cv2.LINE_AA)
            else:
                self.__images.append(frame)
                last_time = time.time()
                print(f'Added image {len(self.__images)}/{num_of_images}')
                cv2.putText(frame, str(len(self.__images)), (20, 60), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 0, 60), 2, cv2.LINE_AA)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) != -1:
                break
        cap.release()
        cv2.destroyAllWindows()

    def calibrate(self):
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Apply camera calibration operation for images in the given directory path.
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((self.__height*self.__width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.__width,
                               0:self.__height].T.reshape(-1, 2)
        # if square_size is 1.5 centimeters, it would be better to write it as 0.015 meters.
        # Meter is a better metric because most of the time we are working on meter level projects.
        objp = objp * self.__square_size
        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.
        for image in self.__images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.__width, self.__height),
                                                     None)
            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners,
                                            (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
        ret, self.__camera_matrix, self.__dist_coef, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                                                        imgpoints,
                                                                                        gray.shape[::-1],
                                                                                        None, None)

    def saveCoefficients(self, path):
        # Save the camera matrix and the distortion coefficients to given path/file.
        if (self.__camera_matrix is not None) and (self.__dist_coef is not None):
            cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
            cv_file.write('K', self.__camera_matrix)
            cv_file.write('D', self.__dist_coef)
            # note you *release* you don't close() a FileStorage object
            cv_file.release()

    @ staticmethod
    def loadCoefficients(path):
        # Loads camera matrix and distortion coefficients.
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode('K').mat()
        dist_coef = cv_file.getNode('D').mat()

        cv_file.release()
        return [camera_matrix, dist_coef]


if __name__ == '__main__':
    clb = Calibration()
    clb.setImages(3, 40)
    clb.calibrate()
    clb.saveCoefficients('calibration.yaml')
