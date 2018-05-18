#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>
#include <string.h>

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
	ros::Publisher pub = n.advertise<std_msgs::String>("BookID", 1000);
	ros::Rate loop_rate(10);
  	while (ros::ok())
  	{

       Mat frame, frame_grayscale;
 
       // LOAD image
       frame = imread("/home/surabhi/zbar_ws/src/zbar/src/rack1.png", CV_LOAD_IMAGE_COLOR);   // Read the file "image.jpg".
 
       if(! frame.data )  // Check for invalid input
       {
              cout <<  "Could not open or find the image" << std::endl ;
              return -1;
       }

    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

//    for (;;) 
//    {
        cvtColor(frame, frame_grayscale, CV_BGR2GRAY);
        int width = frame_grayscale.cols;
        int height = frame_grayscale.rows;
        uchar *raw = (uchar *)(frame_grayscale.data);
        Image image(width, height, "Y800", raw, width * height);
        scanner.scan(image);

        // Extract results
        int counter = 0;
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) 
        {
            time_t now;
            tm *current;
            now = time(0);
            current = localtime(&now);

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

            counter++;
        }

        // Show captured frame, now with overlays!
        imshow("captured", frame);
                                                                                                                                                          
        // clean up
        image.set_data(NULL, 0);
        //waitKey(27);
//cout<<name<<endl;
//    }
    std_msgs::String msg;
    msg.data = name;
    
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  	}
  	return 0;
}
