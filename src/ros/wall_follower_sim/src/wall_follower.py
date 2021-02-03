#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from controller import PID

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan"
    DRIVE_TOPIC = "cmd_vel"
    SIDE = -1 # -1 right is and +1 is left
    VELOCITY = 0.6
    DESIRED_DISTANCE = 0.5

    def __init__(self):
        # Create a node that 
        #   Subscribes to the laser scan topic,
        #   Publishes to  drive topic - to move the vehicle.
        # Initialize subscriber to laser scan.
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.LaserCb)

        # Initialize a publisher of drive commands.
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, Twist, queue_size = 10)

        # Variables to keep track of drive commands being sent to robot.
        self.seq_id = 0

        # Class variables for following.
        self.side_angle_window_fwd_ = math.pi*0.1
        self.side_angle_window_bwd_ = math.pi - math.pi*0.3

        self.point_buffer_x_ = np.array([])
        self.point_buffer_y_ = np.array([])
        self.num_readings_in_buffer_ = 0
        self.num_readings_for_fit_ = 2
        self.reject_dist = 0.7

        self.steer_cmd = 0
        self.vel_cmd = self.VELOCITY

        self.pid = PID()
    
    def GetLocalSideWallCoords(self, ranges, angle_min, angle_max, angle_step):
        # Slice out the interesting samples from our scan. pi/2 radians from pi/4 to (pi - pi/4) radians for the right side.
        positive_start_angle = self.side_angle_window_fwd_
        positive_end_angle   = self.side_angle_window_bwd_
        if self.SIDE == -1: #"right":
            start_angle = -positive_start_angle
            end_angle   = -positive_end_angle
        elif self.SIDE == 1: #"left":
            start_angle = positive_start_angle
            end_angle   = positive_end_angle  

        start_ix_ = int((-angle_min +start_angle)/angle_step)
        end_ix_ = int((-angle_min +end_angle)/angle_step)
        start_ix = min(start_ix_,end_ix_)
        end_ix = max(start_ix_,end_ix_)
        side_ranges = ranges[min(start_ix,end_ix):max(start_ix, end_ix)]
        x_values = np.array([ranges[i]*math.cos(angle_min+i*angle_step) if i < len(ranges) else  ranges[(i - len(ranges))]*math.cos(angle_min+(i - len(ranges))*angle_step) for i in range(start_ix, end_ix) ])
        y_values = np.array([ranges[i]*math.sin(angle_min+i*angle_step) if i < len(ranges) else ranges[i - len(ranges)]*math.sin(angle_min+(i - len(ranges))*angle_step) for i in range(start_ix, end_ix)])

        # Check that the values for the points are within 1 meter from each other. Discard any point that is not within one meter form the one before it.

        out_x = []
        out_y = []
        for ix in range(0, len(x_values)):
            new_point = (x_values[ix],y_values[ix])
            # This conditional handles points with infinite value.
            if  side_ranges[ix] < 2.5 and side_ranges[ix] > 0 and abs(new_point[1]) < 7000 and abs(new_point[0]) < 7000 :
                out_x.append(new_point[0])
                out_y.append(new_point[1])


        return np.array(out_x), np.array(out_y)





    def LaserCb(self, scan_data):
        # This function is called every time we get a laser scan.

        # This is the plan:
        # * Get scan data.
        # * Convert it to x,y coordinates in the local frame of the robot.
        # * Find a least squares - best fit line through those points with numpy.
        #   Consider using data from multiple scans in one least-squares fit cycle.

        #   This is a line equation, with respect to the car at (0,0), with the x axis being the heading.
        #   Get vector theta for the line, and theta_0 as the y intersection.
        # * Find the distance from the line to the origin with ( theta_T dot [[0],[0]] + theta_0 ) / (norm theta)
        # TLDR, We have a vector theta for the line we have found, and a distance to that wall.

        # TODO(yorai): Handle erroneous scan values. If one is too big, or too small, use past value. 
        # Do not do this for too many in a row, maybe just throw scan away if too many corrections.

        angle_step = scan_data.angle_increment
        angle_min = scan_data.angle_min
        angle_max = scan_data.angle_max
        ranges = scan_data.ranges

        # Get data for side ranges. Add to buffer.
        wall_coords_local = self.GetLocalSideWallCoords(ranges, angle_min, angle_max, angle_step)
        #######
        #Find mean and throw out everything that is 1 meter away from mean distance, no outliers.
        # If one differs by more than a meter from the previous one, gets thrown out from both x and y. Distnaces as we go along vector of points.
        # but print the things first.
        #######
        self.point_buffer_x_ = np.append(self.point_buffer_x_, wall_coords_local[0])
        self.point_buffer_y_ = np.append(self.point_buffer_y_, wall_coords_local[1])
        self.num_readings_in_buffer_ +=1

        # If we have enough data, then find line of best fit.
        if self.num_readings_in_buffer_ >= self.num_readings_for_fit_:
            # Find line of best fit.
            # self.point_buffer_x_ = np.array([0, 1, 2, 3])
            # self.point_buffer_y_ = np.array([-1, 0.2, 0.9, 2.1])
            A = np.vstack([self.point_buffer_x_, np.ones(len(self.point_buffer_x_))]).T
            m, c = np.linalg.lstsq(A, self.point_buffer_y_, rcond=0.001)[0]

            # Find angle from heading to wall.

            # Vector of wall. Call wall direction vector theta.
            th = np.array([[m],[1]])
            th /= np.linalg.norm(th)
            # Scalar to define the (hyper) plane
            th_0 = c

            # Distance to wall is (th.T dot x_0 + th_0)/(norm(th))
            dist_to_wall = abs(c/np.linalg.norm(th))

            # Angle between heading and wall.
            angle_to_wall = math.atan2(m, 1)

            # Clear scan buffers.
            self.point_buffer_x_=np.array([])
            self.point_buffer_y_=np.array([])
            self.num_readings_in_buffer_ = 0

            # Simple Proportional controller.
            # Feeding the current angle ERROR(with target 0), and the distance ERROR to wall. Desired error to be 0.
            print("ANGLE", angle_to_wall, "DIST", dist_to_wall)
            steer = self.pid.GetControl(0.0 - angle_to_wall, self.DESIRED_DISTANCE - dist_to_wall, self.SIDE)

            # Publish control to /drive topic.
            drive_msg = Twist()

            # drive_msg.header.seq = self.seq_id
            # self.seq_id += 1

            # Populate the command itself.
            drive_msg.linear.x = self.VELOCITY
            drive_msg.angular.z = steer

            # drive_msg.drive.steering_angle = steer
            # drive_msg.drive.steering_angle_velocity = 0.1
            # drive_msg.drive.speed = self.VELOCITY
            # drive_msg.drive.acceleration = 1
            # drive_msg.drive.acceleration = 0.5
            self.drive_pub.publish(drive_msg)







 




if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
