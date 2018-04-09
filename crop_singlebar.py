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


def rotate_image(mat, angle):
    height, width = mat.shape[:2]
    image_center = (width / 2, height / 2)

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1)

    radians = math.radians(angle)
    sin = math.sin(radians)
    cos = math.cos(radians)
    bound_w = int((height * abs(sin)) + (width * abs(cos)))
    bound_h = int((height * abs(cos)) + (width * abs(sin)))

    rotation_mat[0, 2] += ((bound_w / 2) - image_center[0])
    rotation_mat[1, 2] += ((bound_h / 2) - image_center[1])

    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat


def crop(image_path, coords, saved_location):
    """
    @param image_path: The path to the image to edit
    @param coords: A tuple of x/y coordinates (x1, y1, x2, y2)
    @param saved_location: Path to save the cropped image
    """
    image_obj = Image.fromarray(resizelist[val])
    cropped_image = image_obj.crop(coords)
    #cropped_image.save(saved_location)
    cropped_image.show()


    


# CROP THE BARCODES 
def crop_minAreaRect(img, rect):

    # rotate img
    angle = rect[2]
    rows,cols = img.shape[0], img.shape[1]
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    img_rot = cv2.warpAffine(img,M,(cols,rows))
    cv2.imshow("img_rot", img_rot)
    #img_rot = rotate_image(img, angle)

    # rotate bounding box
    #rect0 = (rect[0], rect[1], 0.0)
    box = cv2.cv.BoxPoints(rect)
    pts = np.int0(cv2.transform(np.array([box]), M))[0]    
    pts[pts < 0] = 0
 
    print(pts[1][1])
    
    # crop
    img_crop = img_rot[ (pts[1][1]+0):(pts[0][1]-0), (pts[1][0]+5):(pts[2][0]-5) ]
    #crop(image, (pts[1][1], pts[0][1], pts[1][0], pts[2][0]), 'crop1ped.jpg')

    cv2.imshow("cropped", img_crop)
    #return img_crop

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
 lower_red = np.array([000,49,106])
 upper_red = np.array([255,106,163])
 # Threshold the HSV image to get only red colors
 mask = cv2.inRange(hsv, lower_red, upper_red)
 # Bitwise-AND mask and original image
 # res is thresholded image
 res = cv2.bitwise_and(image,image, mask= mask) 

 # SOME MORPHOLOGICAL OPERATIONS
 # Apply MeanShiftFiltering. This reduces the number of unnecessary contours in your image. 
 # Note: contours are formed due to small irregularities in apparently uniform object. 
 res = cv2.pyrMeanShiftFiltering(res, 21, 91)
 #cv2.imshow("res", res)

 res = cv2.dilate(res, None, iterations = 4)

 # erode to remove small noise 
 edges = cv2.erode(res, None, iterations = 11) 
 #cv2.imshow("edges0", edges)
 # dilate to make the reqd contours big in size to overcome the shrink due to eroding it previously 
 edges = cv2.dilate(edges, None, iterations = 6)
 #cv2.imshow("edges1", edges) 
 # erode to remove small noise and extra dilating 
 blur = cv2.erode(edges, None, iterations = 3)
 #cv2.imshow("blur", blur)

 #   CONTOURS 
 # compute canny edges for closed boundaries with 100 as minvalue and 200 as maxvalue 
 edges = cv2.Canny(blur,100,200)
 #cv2.imshow("edges", edges)
 # find the external contours  
 contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

 # DECLARATION 
 # list to store the centroids of contours of interest  
 roi_centroids = []
 # list to store the contours of interest
 roi = [] 
 roi_box = []

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
    if (extent >= 0.7):

       # draw a blue 'nghien' rectangle
       cv2.drawContours(image, [box], 0, (255, 0, 0))
       # calculating the moments to compute centroid of the contours 
       M = cv2.moments(ct) 
       # Rounding the centroid to integer
       centroid = ( int(round(M['m10']/M['m00'])), int(round(M['m01']/M['m00'])) )
       # store the centroids of all the contours of interest  
       roi_centroids.append(centroid)
       # Drawing a black little empty circle in the centroid position for all contours
       #cv2.circle(image,centroid,5,(0,0,0),1)
       # store all the bounding boxes of interest  
       roi.append(rect)
       roi_box.append(box)

 # JOINING THE CENTROIDS OF CORRESPONDING BARCODES 
 # 'i' is the centroid in consideration
 i=6
 #for i in range (len(roi_centroids)):
   # 'j' are all the centroids 
 
 for j in range (len(roi_centroids)):
     # consider all the other centroids except the centroid in consideration 
        if (j!=i):
         # The slopes of all lines from 'i'th centroid to 'j'th centroid 
         theta_rad = math.atan2( (roi_centroids[i][1] - roi_centroids[j][1]) , (roi_centroids[i][0] - roi_centroids[j][0]) )
         theta = ( theta_rad * 180 ) / math.pi
         print('theta',theta)
         print('before = ', roi[i][2])
         if ( abs(roi[i][2]) > 90 ):
             roi[i][2] = 90 - abs(roi[i][2]) 
        
         print('after', roi[i][2])
         # difference between the theta of (lines joining the centroid under consideration to all other centroids) 
         # and (the angle of bounding box of the contour of the centroid under consideration) 
         # roi[i][2] is the angle of the bounding box of the centroid under consideration 
         diff = abs(abs(theta) - abs(roi[i][2]))
         
         # If theta is almost equal to angle[i]
         if (diff < 3 ):
             # startpoint is the centroid in consideration 
             startpt_x = roi_centroids[i][0]
             startpt_y = roi_centroids[i][1]
             # endpoint is the centroid for which the slope of line drawn (between the start and end points) 
             # is almost equal to the slope of the bounding box of the contour of the centroid under consideration  
             endpt_x = roi_centroids[j][0]
             endpt_y = roi_centroids[j][1]

             cv2.circle(image,(startpt_x, startpt_y),5,(0,0,0),1)
             cv2.circle(image,(endpt_x, endpt_y),5,(0,0,0),1)

             cv2.circle(image,tuple(roi_box[i][0]),5,(0,0,0),1)
             #cv2.circle(image,tuple(roi_box[i][1]),5,(0,0,0),1)
             cv2.circle(image,tuple(roi_box[j][2]),5,(0,0,0),1)
             cv2.circle(image,tuple(roi_box[j][3]),5,(0,0,0),1)
 
             # draw line 
             #cv2.line(image,(startpt_x,startpt_y),(endpt_x,endpt_y),(255,0,0),3)

             # bounding boxes of the barcodes 
             # storing the rectangle corner points as numpy array to draw the bounding rectangle 
             barcodes = np.array([[roi_box[i][0]], [roi_box[i][1]], [roi_box[j][2]], [roi_box[j][3]]])
             rect = cv2.minAreaRect(barcodes)
             box = cv2.cv.BoxPoints(rect) 
             box = np.int0(box)  
             # draw a red 'nghien' rectangle

             diag1 = math.hypot(roi_box[i][0][0] - roi_box[j][2][0], roi_box[i][0][1] - roi_box[j][2][1])
             diag2 = math.hypot(roi_box[i][1][0] - roi_box[j][3][0], roi_box[i][1][1] - roi_box[j][3][1])
             #if(abs(diag1-diag2) < 2):
                 #cv2.drawContours(image, [box], 0, (0, 0, 255))
             
########################################
             #cv2.imshow("Image", image)
             #print(i)
             #cv2.waitKey(500)

             cropped = crop_minAreaRect(image, rect)
             
             

 # show the final image 
 cv2.imshow("Image", image)

 # repeat the program  
 c = cv2.waitKey(1)
 if c==27:
   break
