//Program to count the no. of books with qr codes that have been encountered during scanning of books in shelf

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//notice iostream does not end in .hpp meaning it is not a header file, instead an i/o class in c++
#include <iostream>
 
using namespace cv;
using namespace std;


//char* argv[] can also be written as char **argv 
int main(int argc, char* argv[])
{	
	VideoCapture Cap(0);
	if (Cap.isOpened()==0) 
	{
		cout<< "Cannot open video cam" << endl;
		return -1;
	}
	int count = 0;
	while(1)
	{	
		Mat frame;
		bool success = Cap.read(frame);
		if (!success)
		{
			cout<< "Cannot read frames from the video stream" << endl;
			break;
		}
		/* Code for processing of one frame */
		//imshow("Source_frame", frame);

    	///Separating region of interest
		Rect Rec(30, 80, 200, 300);//Rect Rec(par1, par2, par3, par4); //par1: x-cordinate, par2: y-cordinate, par3: no of columns, par4: no of rows
		rectangle(frame, Rec, Scalar(255), 1, 8, 0);
		imshow("Source_frame_with_roi", frame);
		Mat Roi = frame(Rec);
		//imshow("Region_of_interest", Roi);

		///Conversion to grayscale
		Mat gray;
		cvtColor(Roi, gray, CV_BGR2GRAY);

		///Binary thresholding of ROI to detect qr codes from rest of the image 
    	Mat thr;
    	///Otsu thresholding:
    	threshold(gray, thr, 175, 255, CV_THRESH_BINARY);
    	//threshold(gray, thr, 200, 255, CV_THRESH_BINARY); //Threshold to detect the white part of the qr code which lies between 200-255
    	//imshow("Thresholded_image", thr);

    	///Morphological operations to remove noise
    	//size is kept to 3 to prevent too much eroding which will render the qrcode boundary unclosed.
    	//Point(-1,-1) is the default anchor point of the element(centre)
    	erode(thr,thr, getStructuringElement(MORPH_ELLIPSE, Size(3,3)),Point(-1,-1),4); 
    	dilate(thr,thr, getStructuringElement(MORPH_ELLIPSE, Size(3,3)),Point(-1,-1),6);
    	erode(thr,thr, getStructuringElement(MORPH_ELLIPSE, Size(3,3)),Point(-1,-1),4);
    	imshow( "Thresholded_&_Morphed", thr );

    	///Counting the no. of largest contours in each frame to find the count of all the qrcodes detected.
    	int largest_area=0;
 		int largest_contour_index=0;
 		Rect bounding_rect;
 		vector<vector<Point> > contours; // Vector for storing contours
 		vector<vector<Point> > contours1;// Vector for storing contours of approximated polygons from previously obtained contours
    	vector<Vec4i> hierarchy;
		double epsilon = 7.0; //parameter used in approximating the polygon

    	//Applying Canny to get clear cut edges:
    	Mat dst;
    	Canny(thr, dst, 50, 200, 3); 
    	///show canny output:
    	//namedWindow("canny_output", CV_WINDOW_AUTOSIZE);
    	//imshow("canny_output", dst);

    	//CV_RETR_EXTERNAL retrieves only the extreme outer contours. 
    	findContours(dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
    	//cout << "No. of contours:" << contours.size() << endl;
    	contours1.resize(contours.size()); // Contour size should be same for transferring contours. If not, it gives segementation fault.
  	    for(int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
    	{	
    		approxPolyDP(contours[i], contours1[i], epsilon, true);
        	if (contours1[i].size()!=4) // Discard the polygon contour if it isn't 4 sided
            {
            	continue;
            }
        	if (!cv::isContourConvex(contours1[i])) // Discard the polygon contour if it isn't convex
            {
            	continue;
            }
       		double a=contourArea( contours1[i],false);//  Find the area of contour
       		if(a>largest_area)
       		{
       			largest_area=a;
       			largest_contour_index=i;                //Store the index of largest contour
       			bounding_rect=boundingRect(contours1[i]); // Find the bounding rectangle for biggest contour
       		}
      	}
		Scalar color(255,255,255);
 		drawContours(dst, contours1, largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
 		imshow("Detected_contour", dst);
 		//***you won't be able to see the bounding rect unless you draw it on a 3 channel image since your bounding rectangle is green in color***//
 		rectangle(Roi,bounding_rect,Scalar(0,255,0),1,8,0);
 		//Bounding Box Centroid
 		Point center = Point(bounding_rect.x + (bounding_rect.width)/2, bounding_rect.y + (bounding_rect.height)/2);
 		circle(Roi,center,4,Scalar(255,0,200),-1,8,0);
 		line(Roi, Point(90,0), Point(90,300), Scalar(0,0,255),1,8,0);
 		imshow("Qr_Code_detected", Roi);
 		//Loop to maintain count
 		if(center.x > 86 && center.x < 90)
 		{
 			count++;
 		}
    	cout<< "No of books:" << count << endl;  

    	if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
    	{  
    		cout << "esc key is pressed by user" << endl;  
    		break;   
    	}
	}
	return 0;
}