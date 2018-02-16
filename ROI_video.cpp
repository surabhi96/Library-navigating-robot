//Code to extract the region of interest from a video(camera feed)

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>  

using namespace cv;
using namespace std;

//char* argv[] can also be written as char **argv 
int main(int argc, char* argv[])
{
	VideoCapture Cap(1);
	if (Cap.isOpened()==0) 
	{
		cout<< "Cannot open video cam" << endl;
		return -1;
	}
	while(1)
	{
		Mat image;
		bool success = Cap.read(image);
		if (!success)
		{
			cout<< "Cannot read frames from the video stream" << endl;
			break;
		}
		// Here goes the code for processing of one frame 
		imshow("Source_frame", image);
		
    	//Rect Rec(par1, par2, par3, par4); //par1: x-cordinate, par2: y-cordinate, par3: no of columns, par4: no of rows
		Rect Rec(30, 80, 200, 300);
		rectangle(image, Rec, Scalar(255), 1, 8, 0);
		imshow("Frame_with_rectangle", image);
		Mat Roi = image(Rec);
		imshow("Region_of_interest", Roi);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
    	{  
    		cout << "esc key is pressed by user" << endl;  
    		break;   
    	}
	}
	return 0;
}