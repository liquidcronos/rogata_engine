#!/usr/bin/env python
import sys
import cv2
import cv2.aruco as aruco
import numpy as np

import rogata_library as rgt


def calibrate_colors(image):
    """Utility to calibrate the colors for contour detection

    Allows the visual calibration of contours which can be saved by pressing the s key.
    Colors are defined in HSV color space.
    For each Value H, S and V the median value as well as the acceptable range can be defined.
    Additionally the ID of a aruco marker used to specify the wanted contour  can be specified.
    Since this can sometimes lead to the contour being the outline of the marker,
    the minimum hole size in pixels can be specified.
    """

    def nothing(_):
        pass
    cv2.namedWindow("Test image")
    cv2.createTrackbar("H", "Test image", 0, 179, nothing)
    cv2.createTrackbar("H range", "Test image", 0, 50, nothing)
    cv2.createTrackbar("S", "Test image", 0, 255, nothing)
    cv2.createTrackbar("S range", "Test image", 0, 120, nothing)
    cv2.createTrackbar("V", "Test image", 0, 255, nothing)
    cv2.createTrackbar("V range", "Test image", 0, 120, nothing)
    cv2.createTrackbar("Marker Id", "Test image", 0, 120, nothing)
    hight, width, _ = image.shape
    cv2.createTrackbar("Minimum contour size", "Test image",
                       0, hight*width, nothing)
    while(1):

        mid_color = np.array([cv2.getTrackbarPos("H", "Test image"),
                              cv2.getTrackbarPos("S", "Test image"),
                              cv2.getTrackbarPos("V", "Test image")])

        step = np.array([cv2.getTrackbarPos("H range", "Test image"),
                         cv2.getTrackbarPos("S range", "Test image"),
                         cv2.getTrackbarPos("V range", "Test image")])

        marker_id = cv2.getTrackbarPos("Marker Id", "Test image")

        min_size = cv2.getTrackbarPos('Minimum contour size', 'Test image')

        min_value = np.zeros(3)
        min_value[0] = -179
        max_value = np.array([179, 255, 255])
        lower_bound = np.clip(mid_color-step, min_value, max_value)
        upper_bound = np.clip(mid_color+step, min_value, max_value)

        used_img = image.copy()
        hsv_img = cv2.cvtColor(used_img, cv2.COLOR_BGR2HSV)

        find_contour = rgt.detect_area(
            hsv_img, lower_bound, upper_bound, marker_id, min_size, True)
        natural_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2BGR)
        cv2.imshow("Test image", natural_img)

        k = cv2.waitKey(1)
        if k == 27:
            break
        if k == ord('s'):
            print("Please Enter a File name:")
            file_name = raw_input()
            np.save(file_name, find_contour)
            print("File ", file_name, " saved.")


if __name__ == "__main__":
    image = cv2.imread(sys.argv[1])
    calibrate_colors(image)
