#!/usr/bin/env python3
import numpy as np
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64


class stanley:
    def get_lateral_stanley(self,current_xy,current_yaw,current_speed,next_waypoint,k,ks):
        '''
        Stanley, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        current_speed: float
            Current speed of the vehicle
        next_waypoint: np.array of floats, shape=3
            Next waypoint for the vehicle to reach [x,y,yaw_path]
        k = 0.001
        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        yy=-current_xy[1]+next_waypoint[1]
        xx=-current_xy[0]+next_waypoint[0]
        alpha = math.atan2 (yy, xx) - current_yaw 
        Id = np.sqrt (yy**2 + xx**2)
        e = Id * np.sin(alpha)
        ans = (alpha + math.atan2(k*(e), current_speed+ks))

        
        if (ans>np.pi):
            ans =  ans - 2*np.pi 
        
        return ans
        pass