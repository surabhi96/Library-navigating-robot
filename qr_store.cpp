// Stores and publishes books corresponding to every rack //

// Editor: Surabhi Verma 
// Last Edit: 21st May 2018

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <iostream>
#include <iterator>
#include <utility>
#include <set> 
#include "dataset_create/book_list.h"

using namespace std;
bool rack_flag = 0;
// set declaration 
set<string> myset;

// iterator declaration for print 
std::set<string>::iterator it1 = myset.begin();

// set return type declaration 
pair< set<string>::iterator, bool> ptr;

// string vector initialization 
dataset_create::book_list book_msg; 

// Book input callback function
void bookinputCallback(const std_msgs::String::ConstPtr& msg) 
{     
     // Insert the incoming book names in myset
     ptr = myset.insert(msg->data.c_str()); 

     // If rack storage of books is complete 
     if (rack_flag = 1) {

          // store myset elements in a vector 
          for (it1 = myset.begin(); it1!=myset.end();  ++it1)
               book_msg.book_vector.push_back(*it1);

          // Initialize publisher Publish set of books in a rack on the topic "book_set"
          ros::NodeHandle nh;
          ros::Publisher pub_book_set = nh.advertise<dataset_create::book_list>("book_set", 100);
  
          // publish vector 
          pub_book_set.publish(book_msg);

          // Indicate operation over. Move to storage of next rack books.
          rack_flag = 0;
     }
}

// Rack input callback function
void rackinputCallback(const std_msgs::Int8::ConstPtr& msg) 
{    
     // Indicates that a rack has been reached  
     rack_flag = 1;
}
 
// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_store");
  ros::NodeHandle n;
  
  // Subsribe to the topic "BookID" which publishes book names 
  ros::Subscriber sub_book = n.subscribe("BookID", 100, bookinputCallback);

  // Subsribe to the topic "Rack_num" which publishes rack numbers 
  ros::Subscriber sub_rack = n.subscribe("Rack_num", 100, rackinputCallback);

  ros::spin();
  return 0;
}
