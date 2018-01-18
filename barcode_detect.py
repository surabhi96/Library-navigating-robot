# USAGE
# python detect_barcode.py --image images/barcode_01.jpg

# import the necessary packages
import numpy as np
import argparse
import cv2
import math

while True:
 # construct the argument parse and parse the arguments
 ap = argparse.ArgumentParser()
 ap.add_argument("-i", "--image", required = True, help = "path to the image file")
 args = vars(ap.parse_args())

 # load the image and convert it to grayscale
 image = cv2.imread(args["image"])
 gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

 # compute the Scharr gradient magnitude representation of the images
 # in both the x and y direction
 gradX = cv2.Sobel(gray, ddepth = cv2.cv.CV_32F, dx = 1, dy = 0, ksize = -1)
 gradY = cv2.Sobel(gray, ddepth = cv2.cv.CV_32F, dx = 0, dy = 1, ksize = -1)
 mean = (gradX + gradY)/2
 #mean = math.sqrt(math.pow(gradX,2) + math.pow(gradY,2))
 #cv2.imshow("gradX", gradX)
 #cv2.imshow("gradY", gradY)
 cv2.imshow("mean", mean)
 # subtract the y-gradient from the x-gradient
 #gradient = cv2.subtract(gradX, gradY)
 gradient = cv2.convertScaleAbs(mean) 

 #cv2.imshow("gradient", gradient)

 # blur and threshold the image
 blurred = cv2.blur(gradient, (7, 7))
 #(_, thresh) = cv2.threshold( blurred, 225, 255, cv2.THRESH_BINARY)
 ret3,th3 = cv2.threshold(blurred,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
 cv2.imshow("thresh", th3)

 # construct a closing kernel and apply it to the thresholded image
 kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
 closed = cv2.morphologyEx(th3, cv2.MORPH_CLOSE, kernel)

 # perform a series of erosions and dilations
 closed = cv2.erode(closed, None, iterations = 4)
 closed = cv2.dilate(closed, None, iterations = 4)
 
 cv2.imshow("closed", closed)
 # find the contours in the thresholded image, then sort the contours
 # by their area, keeping only the largest one
 (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
 c = sorted(cnts, key = cv2.contourArea, reverse = True)[0]

 # compute the rotated bounding box of the largest contour 
 rect = cv2.minAreaRect(c)
 box = np.int0(cv2.cv.BoxPoints(rect))

 # draw a bounding box arounded the detected barcode and display the
 # image
 cv2.drawContours(image, [box], -1, (0, 255, 0), 3)
 cv2.imshow("Image", image)
 c = cv2.waitKey(1)
 if c==27:
   break
 


