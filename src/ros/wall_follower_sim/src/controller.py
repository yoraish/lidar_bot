#!/usr/bin/env python2

import numpy as np
import math
import matplotlib.pyplot as plt
import time

import rospy

class PID():
    def __init__(self):
        # Set gains.
        self.Kp = 1.0
        self.Ki = 0.5
        self.Kd = 0.02
        self.sample_time = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0

        self.dist_weight_ = 0.7
        self.angle_weight_ = 1.6


        self.current_time = time.time()
        self.last_time = self.current_time

        self.last_error = 0.0


    def GetControl(self,angle_error, dist_error, mode = -1):
        # Takes in the pose of the robot wrt the wall, and returns a steer angle control.
        # Mode is the position of the wall. -1 for right and +1 for left.
        
        # Error will be positive if too far away from wall, and if angled away from wall.
        # Error will be negative if too close to wall and if angled towards wall.
        error = mode*self.angle_weight_*angle_error + self.dist_weight_*dist_error

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

                # Remember last time and last error for next calculation
                self.last_time = self.current_time
                self.last_error = error

                steer_endpoint = math.pi/3
                angle_command = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
                angle_command = np.clip(angle_command, -steer_endpoint, steer_endpoint)
                # print("Command", angle_command)
                print_lst = ['-' for i in range(20)]
                print_lst[int((angle_command+steer_endpoint)/(2*steer_endpoint)*len(print_lst)-1)] = '*'
                # print(print_lst)
                return -angle_command*mode

