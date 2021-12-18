#!/usr/bin/env python

import cv2
import numpy as np

# theta = np.arctan((239-235.)/(299-225))
# beta = np.sqrt((355.48297119140625-281.9783935546875)**2+(212.82518005371094-211.4365692138672)**2)/0.1
# ty = (220 - 320)/beta
# tx = (50 - 240)/beta

theta = -0.09
beta = 750.0
tx = -0.297435628054
ty = -0.0763807206584

# Function that converts image coord to world coord
# Note: input x corresponds to columns in the image, input y is rows in the image
def IMG2W(x,y):
    O_c=320
    O_r=240
    xc=(y-O_r)/beta
    yc=(x-O_c)/beta

    cen = np.array( [ [xc], [yc] ] )
    trans = np.array( [ [-0.290717],[-0.105387] ] )
    world = (cen - trans)

    xw = world[0,0]
    yw = world[1,0]

    W = np.array([[xw],[yw]])

    return W

def blob_search(image_raw, color):
    params = cv2.SimpleBlobDetector_Params()
    print("reached")

    params.filterByColor = False
    params.blobColor = 255

    params.filterByArea = False
    params.minArea = 50

    params.filterByCircularity = False

    params.filterByInertia = False

    params.filterByConvexity = False

    detector = cv2.SimpleBlobDetector_create(params)

    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    lower = (0,0,0)     # ice lower
    upper = (0,0,0)   # ice upper
    if color == "yellow":
        lower = (15,100,120)     # yellow lower
        upper = (30,255,255)   # yellow upper

    if color == "ice":
        lower = (70,80,100)   
        upper = (140,255,255)

    

    mask_image = cv2.inRange(hsv_image, lower, upper)

    keypoints = detector.detect(mask_image)

    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    #print(blob_image_center)

    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, mask_image)

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw