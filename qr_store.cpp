#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <iterator>
#include <utility>
#include <set> 
using namespace std;
set<string> myset;
std::set<string>::iterator it1;
//declaring iteration to a set
pair< set<string>::iterator, bool> ptr;
string name;
int countt=0;
int count1=0;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  count1=0;
 // ROS_INFO("I heard: [%s]", msg->data.c_str());
//cout<<"hi"<<endl;
  if (countt<50){
  ptr = myset.insert(msg->data.c_str());
  countt++;
  cout<<countt<<endl;}
  
  else{
      for (it1 = myset.begin(); it1!=myset.end();  ++it1){
        count1++;
        cout<<"yo"<<count1;
        cout << *it1 << " ";}

}
cout<<"bye"<<endl;
}  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_store");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("BookID", 10, chatterCallback);
  
  ros::spin();

  return 0;
}
