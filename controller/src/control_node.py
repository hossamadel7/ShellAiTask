#!/usr/bin/env python3
import numpy as np
import rospy
import math
from classes.stanley import stanley
from classes.pid import PID
from classes.pure_pursuit import pure_pursuit


from std_msgs.msg import Float64MultiArray, Float64



rospy.init_node('control_node')
throtle_pub= rospy.Publisher('controls/throttle', Float64, queue_size=10)
steer_pub = rospy.Publisher('controls/steer', Float64, queue_size=10) 
current_state=None
def update_car_state(data):
    global current_state
    current_state = np.array(data.data)  # [x, y, theta, speed, beta (slip angle), theta_dot]
 
curr_waypoint = None
def update_waypoint(data):
    global curr_waypoint
    curr_waypoint = np.array(data.data)  # [x, y, yaw_path] of next waypoint
 
rospy.Subscriber("vehicle_model/state", Float64MultiArray, update_car_state)
rospy.Subscriber("waypoints", Float64MultiArray, update_waypoint)


pid=PID()
stanley_C=stanley()
pure_pursuit_C=pure_pursuit()
rate=rospy.Rate(30)
kp=rospy.get_param('kp')
kd=rospy.get_param('kd')
ki=rospy.get_param('ki')
k=rospy.get_param('k')
ks=rospy.get_param('ks')

lateral_control_typr = rospy.get_param("lateral_control_type")




while not rospy.is_shutdown():
    rate.sleep()
    if curr_waypoint is None:
        continue
 
    # Getting states variables from current car state (position, heading, speed)
 
    if lateral_control_typr == "stanley":
        steer=stanley_C.get_lateral_stanley(current_state,current_state[2],current_state[3],curr_waypoint,k,ks)
    else:
        steer=pure_pursuit_C.get_lateral_pure_pursuit(current_state,current_state[2],curr_waypoint)
 
    # Longitudinal and lateral control

    throtle=pid.get_longitudinal_control(current_state[3],5,1/30,kp,kd,ki)
    
 
    # Create longitudinal and lateral messages (of type Float64 imported above)
    throtle_pub.publish(throtle)
    steer_pub.publish(steer/10)
    # Publish the 2 messages







