#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace zbar;

// Global variable declaration:
basic_string<char> name;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "zbar_pub");
	ros::NodeHandle n;
	//The zbar publisher node will publish on the topic- BookID
	ros::Publisher pub = n.advertise<std_msgs::String>("BookID", 10);
	ros::Rate loop_rate(1000);
  	while (ros::ok())
  	{
    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        cerr << "Could not open camera." << endl;
        exit(EXIT_FAILURE);
    }
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    namedWindow("captured", CV_WINDOW_AUTOSIZE);
    
    // Create a zbar reader
    ImageScanner scanner;
    
    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    // Capture an OpenCV frame
    cv::Mat frame, frame_grayscale;
    cap >> frame;

    // Convert to grayscale
    cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

    // Obtain image data
    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    // Wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // Scan the image for barcodes
    //int n = scanner.scan(image);
    scanner.scan(image);

    // Extract results
    for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
    {
        time_t now;
        tm *current;
        now = time(0);
        current = localtime(&now);
        // do something useful with results
        /*cout    << "[" << current->tm_hour << ":" << current->tm_min << ":" << setw(2) << setfill('0') << current->tm_sec << "] " << counter << " "
                << "decoded " << symbol->get_type_name()
                << " symbol \"" << symbol->get_data() << '"' << endl;
        */
        //cout << symbol->get_data() << endl;
        //cout << "Location: (" << symbol->get_location_x(0) << "," << symbol->get_location_y(0) << ")" << endl;
        //cout << "Size: " << symbol->get_location_size() << endl;
        name = symbol->get_data();
        // Draw location of the symbols found
        if (symbol->get_location_size() == 4) 
        {
            //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
            line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
            line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
            line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
            line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
        }                          
    }
    // clean up
    //image.set_data(NULL, 0);
    // Show captured frame, now with overlays!
    imshow("captured", frame);

    std_msgs::String msg;
    msg.data = name;
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    ros::spinOnce(); 
    loop_rate.sleep();
    //waitKey(0) will stall the program for infinite period of time
    //Also if you remove waitKey() completely, the display image in imshow wont stall at all.
    waitKey(27);
  	}
  	return 0;
}
