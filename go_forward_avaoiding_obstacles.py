#!/usr/bin/env python

# Written by-Sakshi Kakde
# Date-8 March 2017
# A TurtleBot script that moves the Turtlebot forward. Also the bot will stop if there is some obstacle in front of it.
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python move_paralle.py

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

# For initial movement.
move_cmd_init = Twist()
move_cmd_init.linear.x = 0
move_cmd_init.angular.z = 0

#PUBLISHER
# Create a publisher which can "talk" to TurtleBot and tell it to move parallel
cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

def callback(data):
    distance=data.data
    global move_cmd 
    move_cmd= Twist()
    # let's go forward at 0.2 m/s
    move_cmd.linear.x = 0.1
    # let's turn at 0 radians/s
    move_cmd.angular.z = 0
    if distance <10 :
       rospy.loginfo("I published")
       cmd_vel.publish(move_cmd)


def MoveParallel():
    # initiliaze
    #By putting anonymous=False,by default, rospy registers signal handlers so that it can exit on Ctrl-C
    #Refer-http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
    rospy.init_node('MoveParallel', anonymous=False)
    print "Hi"
    #SUBSCRIBER
    #Subscribes to topic 'chatter' to get distance 
    #the call back function is called when it some data id published on the topic chatter
    rospy.Subscriber("#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# For initial movement.
move_cmd_init = Twist()
move_cmd_init.linear.x = 0
move_cmd_init.angular.z = 0


cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

def callback(data):

    global move_cmd
    move_cmd= Twist()
    if (data.ranges[320]>=0.4):
      print "move"
      move_cmd.linear.x = 0.2
      move_cmd.angular.z =0
    else:
      print "stop"
      move_cmd.linear.x = 0
      move_cmd.angular.z =0

    cmd_vel.publish(move_cmd)


      
   

def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    cmd_vel.publish(Twist())

 

    
def listener():
    rospy.init_node('laserscan', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
     # spin() simply keeps python from exiting until this node is stopped
    cmd_vel.publish(move_cmd_init)
    rospy.on_shutdown(shutdown)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except:
        rospy.loginfo("GoForward node terminated.")/scan", LaserScan, callback)

#
    # Initial movement.
    cmd_vel.publish(move_cmd_init)
    rospy.on_shutdown(shutdown)
    rospy.spin() 

def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    cmd_vel.publish(Twist())

if __name__ == '__main__':
    try:
        MoveParallel()
    except:
        rospy.loginfo("GoForward node terminated.")
