#!/usr/bin/env python3
import numpy as np
import rospy
import math


from std_msgs.msg import Float64MultiArray, Float64


class pure_pursuit:
    def get_lateral_pure_pursuit(self,current_xy,current_yaw,next_waypoint):
        '''
        Pure Pursuit, lateral controller

        Parameters
        ----------
        current_xy: np.array of floats, shape=2
            Current position of the vehicle [x,y] given in CG frame
        current_yaw: float
            Current heading of the vehicle
        next_waypoint: np.array of floats, shape=2
            Next waypoint for the vehicle to reach [x,y]

        Returns
        -------
        steer_angle: float
            Steering angle in rad
        '''
        yy=-current_xy[1]+next_waypoint[1]
        xx=-current_xy[0]+next_waypoint[0]
        alpha=math.atan2(yy,xx) - current_yaw

        Id=math.sqrt(xx**2 +yy**2)
        e=Id*math.sin(alpha)

        return math.atan2( (2*self.L*math.sin(alpha)),Id)
