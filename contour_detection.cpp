#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
using namespace cv;
using namespace std;

Mat frame, gray;

int main(int argc, char* argv[]) 
{
    VideoCapture cap(1); //to change to laptop cam change index to 0
    if (!cap.isOpened())
    {
        cout << "Cannot open the video cam" << endl;  
        return -1;  
    }
    while(1)  
    {  
        bool bSuccess = cap.read(frame); // read a new frame from video  
        if (!bSuccess) //if no success, break loop  
        {  
            cout << "Cannot read a frame from video stream" << endl;  
            break;  
        }
        namedWindow("Original",CV_WINDOW_AUTOSIZE); //create a window called "Original"
        imshow("Original",frame);
        cvtColor(frame, gray, CV_BGR2GRAY);
        namedWindow("Gray",CV_WINDOW_AUTOSIZE); //create a window called "Gray"
        imshow("Gray",gray);

        Canny(gray, gray, 100, 200, 3);
        /// Find contours   
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        RNG rng(12345);
        findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        /// Draw contours
        Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        } 
        imshow("Result window", drawing );
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
        {  
            cout << "esc key is pressed by user" << endl;  
            break;   
        }
    }
    return 0;
}