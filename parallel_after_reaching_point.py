#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64,Bool
import math
import numpy
import time

#INITIALIZATION 
# For initial movement.
move_cmd_init = Twist()
#obstacle distance
ref_dist=25 #in cm
#PID constants
kp=0.06
kd=0.01
ki=0.06
#initializing an arrays of size 1*2
errors=[0]*2
Time=[0]*2
#defining limitaions of the bot
max_zngular_vel=0.4
#PUBLISHER
cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
Time[1]=time.time()
i_error=0


def callback(data):
    if data.data==True:
       rospy.loginfo("yo")
       rospy.Subscriber("chatter", Float64,GetData)
       cmd_vel.publish(move_cmd_init)

def GetData(data):
    distance=data.data
    global move_cmd 
    move_cmd= Twist()
    # let's go forward at 0.1 m/s
    move_cmd.linear.x = 0.1
    #calculating error for controller
    error=distance-ref_dist
    errors[1]=error
    #time difference
    Time[1]=time.time()
    #back differntiation 
    d_error=(errors[1]-errors[0]) / (Time[1]-Time[0])
    i_error=i_error+(errors[1] * Time[1])
    move_cmd.angular.z =kp * error + kd * d_error + ki * i_error#PID 

    #putting limit on angular velocity
    if  move_cmd.angular.z > max_zngular_vel:
        move_cmd.angular.z=max_zngular_vel
    elif move_cmd.angular.z < -max_zngular_vel:
        move_cmd.angular.z=-max_zngular_vel
   
    rospy.loginfo(Time[1]-Time[0]) #printing time interval  
    cmd_vel.publish(move_cmd)#publishing data 
    errors[0]=errors[1]#updating the errors anf Time array
    Time[0]=Time[1]


def Move():
    rospy.init_node('move_parallel', anonymous=False)
    rospy.Subscriber("Flag", Bool, callback)
    rospy.on_shutdown(shutdown)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

def shutdown():
    rospy.loginfo("Stop TurtleBot")# stop turtlebot   
    cmd_vel.publish(Twist()) # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot



if __name__ == '__main__':
    try:
        Move()
    except:
        rospy.loginfo("Node terminated.")

