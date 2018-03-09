#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64




# For initial movement.
move_cmd_init = Twist()
move_cmd_init.linear.x = 0
move_cmd_init.angular.z = 0

ref_dist=10
kp=0.01

cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

def callback1(data):
   global kinect_distance 
   kinect_distance=data.ranges[320]

def callback2(data):

    global move_cmd
    move_cmd= Twist()
    ultrasonic_distance=data.data

    if (kinect_distance>=0.4):
      print "move"
      move_cmd.linear.x = 0.1
      error=ultrasonic_distance-ref_dist
      move_cmd.angular.z =kp * error
      cmd_vel.publish(move_cmd)


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

    rospy.Subscriber("scan", LaserScan, callback1)
    rospy.Subscriber("ultraonic_data", Float64, callback2)
    cmd_vel.publish(move_cmd_init)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.on_shutdown(shutdown)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except:
        rospy.loginfo("move_parallel_without_hitting node terminated.")
