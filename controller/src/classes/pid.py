#!/usr/bin/env python3
import numpy as np
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64

class PID:
    Ui=0
    prev_error=0
    def __init__(self, L=4.9):
        self.L = L
       
    def get_longitudinal_control(self,v_current,v_desired,dt,Kp,Kd,Ki):
        '''
        PID Longitudinal controller
        Parameters
        ----------
        v_current: float
            Current speed of the vehicle
        v_desired: float
            Desired speed of the vehicle
        dt: float
            Delta time since last time the function was called

        Returns
        -------
        throttle_output: float
            Value in the range [-1,1]
        '''
        
        error=v_desired-v_current
        Up=Kp*error
        Ud=Kd*(error-self.prev_error)*(1/dt)
        self.Ui+=Ki*error*dt
        self.prev_error=error

        c=math.tanh(Up+Ud+self.Ui)
        return c