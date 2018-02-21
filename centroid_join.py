# Last Edit: 21st Feb 2018
# Author: Surabhi Verma 

# ASSUMPTION1: Reqd. opencv 2.4.9 and python2
# ASSUMPTION2: The image test4_color.jpeg is in the dataset folder. 
# USAGE: python2 red_detect.py --image dataset/test4_color.jpeg

# import the necessary packages
import numpy as np
# to parse our command line arguments 
import argparse
import cv2
import math
from matplotlib import pyplot as plt

# continuous loop 
while True:

 # COMMAND LINE ARGUMENTS 
 # construct the argument parse and parse the arguments
 ap = argparse.ArgumentParser()
 ap.add_argument("-i", "--image", required = True, help = "path to the image file")
 args = vars(ap.parse_args())

 # THRESHOLDING THE IMAGE 
 # load the image, convert it to HSV for red color thresholding  
 image = cv2.imread(args["image"])
 hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 # decide the upper and lower bounds of your threshold 
 lower_red = np.array([136,69,68])
 upper_red = np.array([255,167,255])
 # Threshold the HSV image to get only red colors
 mask = cv2.inRange(hsv, lower_red, upper_red)
 # Bitwise-AND mask and original image
 # res is thresholded image
 res = cv2.bitwise_and(image,image, mask= mask) 

 # SOME MORPHOLOGICAL OPERATIONS
 # Apply MeanShiftFiltering. This reduces the number of unnecessary contours in your image. 
 # Note: contours are formed due to small irregularities in apparently uniform object. 
 res = cv2.pyrMeanShiftFiltering(res, 21, 91)
 # erode to remove small noise 
 edges = cv2.erode(res, None, iterations = 10) 
 # dilate to make the reqd contours big in size to overcome the shrink due to eroding it previously 
 edges = cv2.dilate(edges, None, iterations = 6) 
 # erode to remove small noise and extra dilating 
 blur = cv2.erode(edges, None, iterations = 3)
 
 # FIND CONTOURS 
 # compute canny edges for closed boundaries with 100 as minvalue and 200 as maxvalue 
 edges = cv2.Canny(blur,100,200)
 # find the external contours  
 contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

 # DECLARATION 
 # list to store the centroids of contours of interest  
 roi_centroids = []
 # list to store the angles of contours of interest
 roi_angles = [] 

 # DRAW BOUNDING BOXES FOR THE CONTOURS 
 for ct in contours:

    # get the min area rect
    # rect[0] gives the (x,y) coordinates of center of rectangle 
    # rect[1] gives the (width,height) of rectangle 
    # rect[2] gives the tilt angle of rectangle     
    rect = cv2.minAreaRect(ct)
    # This function gives a 'list' of 4 corners coordinates of the rectangle
    # (in anticlockwise direction starting from bottom left)
    box = cv2.cv.BoxPoints(rect) 
    # convert all coordinates floating point values to int
    box = np.int0(box)
    # find the area of the contour 
    area = cv2.contourArea(ct)
    # find the area of the bounding box
    rect_area = rect[1][0]*rect[1][1]
    # compute the ratio
    extent = float(area)/rect_area

    # This condition is applied so that only rectangles and not other noise is considered   
    # if the ratio is almost = 1 
    if (extent >= 0.8):

       # draw a red 'nghien' rectangle
       cv2.drawContours(image, [box], 0, (0, 0, 255))
       # calculating the moments to compute centroid of the contours 
       M = cv2.moments(ct) 
       # Rounding the centroid to integer
       centroid = ( int(round(M['m10']/M['m00'])), int(round(M['m01']/M['m00'])) )
       # store the centroids of all the contours of interest  
       roi_centroids.append(centroid)
       # Drawing a black little empty circle in the centroid position for all contours
       cv2.circle(image,centroid,5,(0,0,0),1)
       # angle of the line to be drawn 
       angle = rect[2]
       # store the angles of all the bounding boxes of interest  
       roi_angles.append(angle)

 # JOINING THE CENTROIDS OF CORRESPONDING BARCODES 
 # 'i' is the centroid in consideration 
 for i in range (len(roi_centroids)):
   # 'j' are all the centroids 
   for j in range (len(roi_centroids)):
     # consider all the other centroids except the centroid in consideration 
     if (j!=i):
         # The slopes of all lines from 'i'th centroid to 'j'th centroid 
         theta_rad = math.atan2( (roi_centroids[i][1] - roi_centroids[j][1]) , (roi_centroids[i][0] - roi_centroids[j][0]) )
         theta = ( theta_rad * 180 ) / math.pi
         # difference between the lines joining current centroid to all other centroids 
         # and the angle of bounding box of the contour of current centroid 
         diff = abs(theta - roi_angles[i])

         # If theta is almost equal to angle[i]
         if (diff < 5):
             # startpoint is the centroid in consideration 
             startpt_x = roi_centroids[i][0]
             startpt_y = roi_centroids[i][1]
             # endpoint is the centroid for which the slope of line drawn (between the start and end points) 
             # is almost equal to the slope of the bounding box of the contour of the centroid under consideration  
             endpt_x = roi_centroids[j][0]
             endpt_y = roi_centroids[j][1]
             # draw line 
             cv2.line(image,(startpt_x,startpt_y),(endpt_x,endpt_y),(255,0,0),3)

 # show the final image 
 cv2.imshow("Image", image)

 # repeat the program  
 c = cv2.waitKey(1)
 if c==27:
   break

