//Code to extract the region of interest from an image

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;
using namespace std;

int main(int argc, const char** argv)
{
 Mat image;
 image = imread("book1.jpg", CV_WINDOW_AUTOSIZE);
 imshow("Source_frame", image);
 //separating the region of interest
 Rect Rec(30, 260, 150, 150);//Rect Rec(par1, par2, par3, par4); //par1: x-cordinate, par2: y-cordinate, par3: no of columns, par4: no of rows
 rectangle(image, Rec, Scalar(255), 1, 8, 0);
 imshow("Frame_with_rectangle", image);
 Mat Roi = image(Rec);
 imshow("Region_of_interest", Roi);

/*
 ///Show region of interest against a black background
 //Mat background(image.rows,image.cols,CV_8UC3,Scalar(0,0,0));
 Mat background(240,320,CV_8UC3,Scalar(0,0,0));
 Rect WhereRec(0, 0, Roi.cols, Roi.rows);
 Roi.copyTo(background(WhereRec));
 imshow("Region_of_interest_against_black_background", background);
*/

 waitKey(0);
 return 0;
}

