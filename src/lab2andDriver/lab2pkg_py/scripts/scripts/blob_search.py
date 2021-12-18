#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration [(377.6545104980469, 256.2081604003906)]

theta = 0.0147
bet = 770.68
tx= -0.2069
ty= -0.085

# Function that converts image coord to world coord
# Note: input x corresponds to columns in the image, input y is rows in the image
def IMG2W(x,y):
    global theta
    global bet
    global tx
    global ty
    o_r = 240
    o_c = 320

    r=y
    c=x
    
    
    xc = (r - o_r)/bet
    yc = (c - o_c)/bet
    
    camera=np.array([[(r-o_r)/bet],[(c-o_c)/bet]])
    tvec=np.array([[tx],[ty]])
    w=np.array([[np.cos(theta), -np.sin(theta)], 
     [np.sin(theta), np.cos(theta)]])

    world = np.matmul(np.linalg.inv(w), (camera-tvec))

    xw=world [0] [0]
    yw=world [1] [0]
    
    return (xw, yw)

    
    pass

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    params.blobColor = 120

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 200

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.20
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)
   


    # ========================= Student's code starts here =========================

    if (color == 'green'):
        lower = (35,125,49) #green lower
        upper = (80,255,255)  #green upper

    elif (color == 'blue'):
        lower = (80, 80,40)     # blue lower
        upper = (140,255,255)   # blue upper

    # lower = (80, 90,40)     # blue lower
    # upper = (140,255,255)   # blue upper
    
    # lower2 = (40,130,49) #green lower
    # upper2 = (80,255,255)  #green upper

    # lowerO = (10, 90,40)     # blue lower
    # upperO = (25,255,255)   # blue upper
    
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    # mask_image2 =cv2.inRange(hsv_image, lower2, upper2)
    # mask_image = cv2.inRange(hsv_image, lowerO, upperO) # calibration stick 

    # mask_image += mask_image2

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
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
