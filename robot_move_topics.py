#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
x=0
y=0 
z=0
yaw=0

def poseCallback(pose_message):
    global x
    global y,yaw
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta

def go_to_goal(velocityPub,x_goal,y_goal):
            x0=x
            y0=y
            yaw0=yaw
            
            loop_rate=rospy.Rate(10)
            velocity=Twist()
            while(True):
                B=0.2
                distance=abs(math.sqrt( ( (x_goal-x0) **2) + ( (y_goal-y0)**2) ) )
                linear_speed=B * distance
                Z=1.0
                angle_needed=math.atan2(y_goal-y0,x_goal-x0)
                angular_speed=Z* (angle_needed-yaw0)

                velocity.linear.x=linear_speed
                velocity.angular.z=angular_speed
                velocityPub.publish(velocity)
                loop_rate.sleep()

                if(distance<0.01):
                    break



rospy.init_node("robot")

mover=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
pos_sub=rospy.Subscriber('/turtle1/pose',Pose,poseCallback)


counter=0
rate=rospy.Rate(1)

go_to_goal(mover,10,20)
while not rospy.is_shutdown():
    rate.sleep()