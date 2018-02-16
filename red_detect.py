# USAGE
# python edge_detect_barcode.py --image images/barcode_01.jpg

# import the necessary packages
import numpy as np
import argparse
import cv2
import math
from matplotlib import pyplot as plt

while True:
 # construct the argument parse and parse the arguments
 ap = argparse.ArgumentParser()
 ap.add_argument("-i", "--image", required = True, help = "path to the image file")
 args = vars(ap.parse_args())

 # load the image, remove noise by gaussian blur, convert it to grayscale 
 image = cv2.imread(args["image"])
 hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

 lower_blue = np.array([136,69,68])
 upper_blue = np.array([255,167,255])
 # Threshold the HSV image to get only blue colors
 mask = cv2.inRange(hsv, lower_blue, upper_blue)
 # Bitwise-AND mask and original image
 res = cv2.bitwise_and(image,image, mask= mask)
 cv2.imshow('image',image)
 cv2.imshow("res", res) # show thresholded image 
 res = cv2.pyrMeanShiftFiltering(res, 21, 91)
 #blur = cv2.GaussianBlur(res,(15,15),0) #remove blur as the change in gradient for detecting proper edges is hampered 
 # erode dilate erode 
 edges = cv2.erode(res, None, iterations = 10)
 cv2.imshow("erode1", edges)
 edges = cv2.dilate(edges, None, iterations = 6)
 cv2.imshow("dilate1", edges)
 blur = cv2.erode(edges, None, iterations = 3)
 cv2.imshow("blur", blur)
 
# compute canny edge (draws edges for open/ closed boundaries)
 edges = cv2.Canny(blur,100,200)
 #edges = cv2.dilate(edges, None, iterations = 1)
 cv2.imshow("edges", edges)

 contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 moments  = [cv2.moments(cnt) for cnt in contours]
 # Nota Bene: I rounded the centroids to integer.
 centroids = [( int(round(m['m10']/m['m00'])),int(round(m['m01']/m['m00'])) ) for m in moments]
 
 print "contours %d"%len(contours)
 cv2.drawContours(image, contours, -1, (0, 255, 0), -1) #---set the last parameter to -1
# cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)
# show the blurred image
 for c in centroids:
    # I draw a black little empty circle in the centroid position
    cv2.circle(image,c,5,(0,0,0))
 cv2.imshow("Image", image)




 c = cv2.waitKey(1)
 if c==27:
   break


