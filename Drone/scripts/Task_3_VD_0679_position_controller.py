#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import time
import tf
import math


# Class for position control
class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        print("start")
        rospy.init_node('position_controller')  # initializing ros node with name 'position_controller'

        # This corresponds to gps coordinates of eDrone
        self.drone_gps_coordinates = [0,0,0]
        self.drone_gps_coordinates_meters = [0, 0] # drone coordinates in metres

        # Declaring drone_command of message type edrone_cmd and initializing values
        self.drone_command = edrone_cmd()
        self.drone_command.rcRoll = 1500.0
        self.drone_command.rcPitch = 1500.0
        self.drone_command.rcYaw = 1500.0
        self.drone_command.rcThrottle = 1000.0

        # Declaring marker_cmd of message type MarkerData and initializing values
        self.marker_cmd = MarkerData()
        self.marker_cmd.marker_id = 0
        self.marker_cmd.err_x_m = 0.0
        self.marker_cmd.err_y_m = 0.0

        # Kp, Kd and ki for [latitude, longitude, altitude].
        # tentative altitude values
        self.Kp = [250000, 250000, 150]
        self.Ki = [0.000000624, 0.000000624, 0.006]
        self.Kd = [7150000, 7150000, 3333.33]
        
        # Other necessary variables
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]

        # Other necessary variables
        self.move_to = [0, 0, 0]
        self.landing_found = False

        self.all_next_setpoint = [] # To store the next setpoints

        self.global_marker = [0, 0] # marker location for global coordinates
        self.marker = [0,0] # marker location from drone
        self.i_check = 0 # check the building id and self.i

        self.state = 0 # Store drone state
        self.j = 0 # counter when travelling between setpoints
        self.i = 0 # counter when moving between different destinations

        self.drop_location = [[18.9990965928, 72.0000664814, 10.75], [18.9990965925, 71.9999050292, 22.2], [18.9993675932, 72.0000569892, 10.7]]  # Box location
        self.above_drop_location = []
        for x,y,z in self.drop_location:
            self.above_drop_location.append([x, y, z + 15]) # Coordinates above the box at fixed height

        self.sample_time = 0.06  # This is the time interval to run pid.
    
        # Publishing /drone_command 
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscribing to /edrone/gps and /edrone/marker_data
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        rospy.Subscriber('/edrone/marker_data', MarkerData, self.marker_data_callback)

        time.sleep(2) # time to load all the subscribers

    # Callback function for marker data
    def marker_data_callback(self, msg):
        self.marker[0] = msg.err_x_m
        self.marker[1] = msg.err_y_m
        self.i_check = msg.marker_id


    # Callback function for drone_gps
    def drone_gps_callback(self, msg):
        self.drone_gps_coordinates[0] = msg.latitude
        self.drone_gps_coordinates[1] = msg.longitude
        self.drone_gps_coordinates[2] = msg.altitude

        # Converting latitude and longitude into meters
        self.drone_gps_coordinates_meters[0] = self.lat_to_x(self.drone_gps_coordinates[0])
        self.drone_gps_coordinates_meters[1] = self.long_to_y(self.drone_gps_coordinates[1])


    # To convert latitude to meters
    def lat_to_x(self, input_latitude):
        return 110692.0702932625 * (input_latitude - 19) 


    # To convert longitude to meters
    def long_to_y(self, input_longitude):
        return -105292.0089353767 * (input_longitude - 72)


    # To convert meters to latitude
    def x_to_lat(self, input_x):
        return input_x / 110692.0702932625 + 19


    # To convert meters to longitude
    def y_to_long(self, input_y):
        return input_y / -105292.0089353767 + 72


    # Calculates setpoints based on drone position and desired position
    def get_next_setpoint(self, desired_position):

        # Resetting variables
        self.all_next_setpoint = [] # Contains all the generated setpoints
        self.j = 0

        desired_position_meters = [self.lat_to_x(desired_position[0]), self.long_to_y(desired_position[1])] # Getting desired position in meters

        self.total_distance = math.sqrt((desired_position_meters[0] - self.drone_gps_coordinates_meters[0]) ** 2 + (desired_position_meters[1] - self.drone_gps_coordinates_meters[1]) ** 2) # Calculating distance between desired position and current drone position
        d = int(math.floor(self.total_distance))
        number_of_setpoints = max(int(d / 17), 1)

        p_vector = desired_position_meters[0] - self.drone_gps_coordinates_meters[0], desired_position_meters[1] - self.drone_gps_coordinates_meters[1] # Vector from drone position to the desired position
        p_mag = 1 / float(number_of_setpoints) # Number of partitions of the p_vector

        next_point = self.drone_gps_coordinates_meters

        # add one setpoint when drone is leaving the marker location
        if not self.landing_found:
            self.all_next_setpoint.append([self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], self.drone_gps_coordinates[2] + 7.5])
        self.all_next_setpoint.append([self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], self.above_drop_location[self.i][2]]) # append the current location at height
        
        # get all the setpoints
        for i in range(number_of_setpoints):
            next_point[0] = next_point[0] + p_mag * p_vector[0] 
            next_point[1] = next_point[1] + p_mag * p_vector[1] 

            self.all_next_setpoint.append([self.x_to_lat(next_point[0]),self.y_to_long(next_point[1]), self.above_drop_location[self.i][2]]) # append the next setpoint


    # Checks the drone state and changes it accordingly
    def modify_state(self, desired_position):
        desired_position_meters = (self.lat_to_x(desired_position[0]), self.long_to_y(desired_position[1])) # Getting desired position in meters

        # if state is equal to 4
        if self.state == 4:
            self.move_to[2] = self.drop_location[self.i][2] + 1 # To hover 1 meter above the mark
               
            if self.drone_gps_coordinates[2] - self.move_to[2] < 0.2 and (self.i_check == self.i + 2 or self.i_check == 3): # checking for the height and the values of building id
                self.state = 0
                self.landing_found = False

                self.i += 1 # move to the next marker
                if self.i > 2: # check for last building 
                    self.cmd_pub.publish(1500,1500,1500,1000,0,0,0,0) # publish drone command to stop the drone
                    exit(0)
                e_drone.get_next_setpoint(e_drone.above_drop_location[e_drone.i]) # Generate setpoints to move towards box location

        # If drone has reached the desired position change the state to 1
        elif abs(desired_position[0] - self.drone_gps_coordinates[0]) < 0.0000022585 and abs(desired_position[1] - self.drone_gps_coordinates[1]) < 0.0000022585: # if near destination
            self.state = 3

        # If drone has not reached the desired position change the state to 0
        else:
            self.state = 0


    # get the precise location of the landing marker
    def get_land_location(self):
        # calculating global marker location 
        self.global_marker[0] = self.marker[0] + self.drone_gps_coordinates_meters[0]
        self.global_marker[1] = self.marker[1] + self.drone_gps_coordinates_meters[1]

        self.move_to = [self.x_to_lat(self.global_marker[0]), self.y_to_long(self.global_marker[1]), self.drone_gps_coordinates[2]] # target location


    # PID finction
    def pid(self, setpoint_gps_coordinates):
        error = [0,0,0]
        pid_output = [0,0,0]

        for i in range(len(error)):

            # Calculating errors in the latitude, longitude and altitude.
            error[i] = (setpoint_gps_coordinates[i] - self.drone_gps_coordinates[i])

            # Calculating error_sum for the integral part of PID equation.
            self.error_sum[i] += error[i]

            # Calculating error_change for the differential part of PID equation.
            self.error_change[i] = error[i] - self.prev_error[i]

            # Calculating PID output.
            pid_output[i] = (self.Kp[i] * error[i] + self.Kd[i] * self.error_change[i] + self.Ki[i] * self.error_sum[i])

        # Controlling roll, pitch, throttle and yaw using the PID outputs.
        self.drone_command.rcRoll = 1500 + pid_output[0]
        self.drone_command.rcPitch = 1500 + pid_output[1]
        self.drone_command.rcYaw = 1500
        self.drone_command.rcThrottle = 1500 + pid_output[2]

        # Assigning current error to the previous error.
        self.prev_error = error

        # Publishing the drone_command
        self.cmd_pub.publish(self.drone_command)

        # If desired setpoint is reached
        if abs(setpoint_gps_coordinates[0] - self.drone_gps_coordinates[0]) < 0.0000022585 and abs(setpoint_gps_coordinates[1] - self.drone_gps_coordinates[1]) < 0.0000022585 and abs(setpoint_gps_coordinates[2] - self.drone_gps_coordinates[2]) < 0.3:
            
            self.j += 1 # go the next setpoint

            if self.state == 0 and self.j > len(self.all_next_setpoint) - 1: # if drone has reached building at some height
                self.landing_found = True
                self.state = 3
                self.get_land_location() # getting the marker global location

            elif self.state == 3 and self.j > len(self.all_next_setpoint) - 1: # check if the drone has reached the landmark with height
                self.state = 4
                self.modify_state(self.move_to) 


def main():

    # check for states 
    if e_drone.state == 0:
        e_drone.pid(e_drone.all_next_setpoint[e_drone.j]) # call pid and modify state
        e_drone.modify_state(e_drone.above_drop_location[e_drone.i])

    elif e_drone.state == 3:
        e_drone.pid(e_drone.move_to)# call pid 
    elif e_drone.state == 4:
        e_drone.pid(e_drone.move_to) # call pid and modify state
        e_drone.modify_state(e_drone.move_to)

    else:
        rospy.loginfo('unknown state')


if __name__ == '__main__':

    e_drone = Edrone() # Edrone object creation
    r = rospy.Rate(1/e_drone.sample_time)

    e_drone.get_next_setpoint(e_drone.above_drop_location[e_drone.i]) # Generate setpoints to move towards box location
    while not rospy.is_shutdown():
        try:
            main() # calling main function
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
