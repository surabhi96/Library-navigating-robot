#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(data):
   if (data.ranges[320] >= 0.4):
     print "far"
   else:
     print "close"
 

    
def listener():
    rospy.init_node('laserscan', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
