# Last Edit: 19th Feb 2018
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
 # construct the argument parse and parse the arguments
 ap = argparse.ArgumentParser()
 ap.add_argument("-i", "--image", required = True, help = "path to the image file")
 args = vars(ap.parse_args())

 # load the image, convert it to HSV for red color thresholding  
 image = cv2.imread(args["image"])
 hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 # decide the upper and lower bounds of your threshold 
 lower_red = np.array([136,69,68])
 upper_red = np.array([255,167,255])
 # Threshold the HSV image to get only red colors
 mask = cv2.inRange(hsv, lower_red, upper_red)
 # Bitwise-AND mask and original image
 res = cv2.bitwise_and(image,image, mask= mask)
 # show thresholded image
 cv2.imshow("res", res) 

 # Apply MeanShiftFiltering. This reduces the number of unnecessary contours in your image. 
 # Note: contours are formed due to small irregularities in apparently uniform object. 
 res = cv2.pyrMeanShiftFiltering(res, 21, 91)
 # erode to remove small noise 
 edges = cv2.erode(res, None, iterations = 10)
 cv2.imshow("erode1", edges)
 # dilate to make the reqd contours big in size to overcome the shrink due to eroding it previously 
 edges = cv2.dilate(edges, None, iterations = 6)
 cv2.imshow("dilate1", edges)
 # erode to remove small noise and extra dilating 
 blur = cv2.erode(edges, None, iterations = 3)
 cv2.imshow("blur", blur)
 
 # compute canny edges for closed boundaries with 100 as minvalue and 200 as maxvalue 
 edges = cv2.Canny(blur,100,200)
 # show the output of canny edge 
 cv2.imshow("edges", edges)
 # find the external contours  
 contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

 # draw bounding boxes for all the contours 
 final_contours = []
 for ct in contours:
    # get the min area rect
    # rect[0] gives the (x,y) coordinates of center of rectangle 
    # rect[1] gives the (width,height) of rectangle 
    # rect[2] gives the tilt angle of rectangle     
    rect = cv2.minAreaRect(ct)
    # This function gets 4 corners of the rectangle
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
       cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
       # calculating the moments to compute centroid of the contours 
       M = cv2.moments(ct) 
       # Rounding the centroids to integer
       centroids = ( int(round(M['m10']/M['m00'])), int(round(M['m01']/M['m00'])) )
       # Drawing a black little empty circle in the centroid position for all contours
       cv2.circle(image,centroids,5,(0,0,0),1)
       # angle of the line to be drawn 
       angle = rect[2];
       # length of the line 
       length = 750;
       # start coordinates of the line 
       startpt_x = int(round(rect[0][0]))
       startpt_y = int(round(rect[0][1]))
       # end coordinates of the line 
       endpt_x = int(round(startpt_x + length * math.cos(angle * math.pi / 180.0)))
       endpt_y = int(round(startpt_y + length * math.sin(angle * math.pi / 180.0)))
       # draw line 
       cv2.line(image,(startpt_x,startpt_y),(endpt_x,endpt_y),(255,0,0),10)
       # show the final image 
       cv2.imshow("Image", image)

 # repeat the program  
 c = cv2.waitKey(1)
 if c==27:
   break

