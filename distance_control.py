#!/usr/bin/env python

# A TurtleBot script that moves TurtleBot forward at a fixed distance from the wall. Press CTRL + C to stop.  
# To run on TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python distance_control.py

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

 
class DistanceControl():

    def __init__(self):
         # initialize
            rospy.init_node('DistanceControl', anonymous=False)
	    rospy.loginfo("To stop TurtleBot CTRL + C")    
            rospy.on_shutdown(self.shutdown)        
            self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
            self.move_cmd = Twist()           
            self.counter = 0
            self.kp = 0.1
            self.kd = 0.1
            self.ki = 0
            self.dt = 1
            self.listener()                                
  
    def listener(self):
            #rospy.init_node('listener', anonymous=True)
            ti = rospy.get_time()
            rospy.Subscriber("chatter", Float64, self.callback)
            rate = rospy.Rate(10);

            while not rospy.is_shutdown():
                self.cmd_vel.publish(self.move_cmd)
                rate.sleep()
            tf = rospy.get_time()
            self.dt = tf - ti
            rospy.spin()

    def callback(self, data): 
            #t = rospy.get_time() 

            if (self.counter == 0):
                self.start = data.data # setting reference position 
            self.counter = self.counter + 1
            distance = data.data
            error = distance - self.start 
            self.straight()
            #round_err = int(round((error))
            #self.turn(error)
            if (int(round(abs(error))) > 1):
               self.turn(error)
               rospy.loginfo("turn") 
               

    def turn(self, error):
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = self.kp*error 

    def straight(self):
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 0

    def shutdown(self):
            rospy.loginfo("Stop TurtleBot")
	    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
            self.cmd_vel.publish(Twist())
	    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
            rospy.sleep(1)
 
if __name__ == '__main__':
    try:
            DistanceControl()
    except:
            rospy.loginfo("DistanceControl node terminated.")
