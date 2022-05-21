#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = np.arctan2(1,75)
beta = 770
tx = .24
ty = .11
Oc = -240
Or = -320

# Function that converts image coord to world coord
def IMG2W(col, row):
    x_c = (Oc +row)/beta
    y_c = (Or +col)/beta
    Xw = np.cos(theta)*(x_c+tx) - np.sin(theta)*(y_c+ty)
    Yw = np.sin(theta)*(x_c+tx) + np.cos(theta)*(y_c+ty)
    
    return (Xw, Yw)

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    #params.blobColor = 255

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 250
    #params.minArea = 150
   
    # Filter by Circularity
    params.filterByCircularity = False
    
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
    
    if (color == "green"):
        lower = (40,80, 80)  #green lower
        upper = (80,255,255)    # green upper
    elif(color == "blue"):
        lower = (110,80,80)     # blue 
        upper = (130,255,255)
    elif color == 'orange':
        lower = (0,80,50)     # orange 
        upper = (15,255,255)
    
    #lower = (10,170,170)
    #upper = (60,255,255)
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)
    blob_image_center = []
    # Find blob centers in the image coordinates
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    #print("key" + str(blob_image_center))
    #print("theta" + str(np.arctan(abs(blob_image_center[0][1]-blob_image_center[1][1])/abs(blob_image_center[0][0]-blob_image_center[1][0]))))
    #print("beta" + str(np.sqrt((blob_image_center[0][0]-blob_image_center[1][0])**2+(blob_image_center[0][1]-blob_image_center[1][1])**2)/0.1))
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

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
