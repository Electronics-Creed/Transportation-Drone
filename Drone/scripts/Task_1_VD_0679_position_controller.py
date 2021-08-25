#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf
import math


class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        time.sleep(5)

        rospy.init_node('position_controller')  # initializing ros node with name drone_control

        # This corresponds to gps coordinates of eDrone
        self.drone_gps_coordinates = [0.0, 0.0, 0.0]

        # Declaring drone_command of message type edrone_cmd and initializing values
        self.drone_command = edrone_cmd()
        self.drone_command.rcRoll = 1500.0
        self.drone_command.rcPitch = 1500.0
        self.drone_command.rcYaw = 1500.0
        self.drone_command.rcThrottle = 1000.0

        # Kp, Kd and ki for [latitude, longitude, altitude].
        self.Kp = [246000, 240000, 364.6]
        self.Ki = [0.104 * 1e-3, 0.008 * 1e-3, 0]
        self.Kd = [429000, 450000, 251.4]
        
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]

        # # This is the sample time needed to run pid.
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)

        # Subscribing to /edrone/gps
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)

    # Callback function for drone_gps
    def drone_gps_callback(self, msg):
        self.drone_gps_coordinates[0] = msg.latitude
        self.drone_gps_coordinates[1] = msg.longitude
        self.drone_gps_coordinates[2] = msg.altitude

    def pid(self, setpoint_gps_coordinates):

        error = [0,0,0]
        pid_output = [0,0,0]

        for i in range(len(error)):

            # Calculating errors in the latitude, longitude and altitude.
            error[i] = (setpoint_gps_coordinates[i] - self.drone_gps_coordinates[i])

            # Calculating error_sum for the integral part of PID equation.
            self.error_sum[i] += (error[i] * self.sample_time)

            # Calculating error_change for the differential part of PID equation.
            self.error_change[i] = (error[i] - self.prev_error[i]) / self.sample_time

            # Calculating PID output.
            pid_output[i] = (self.Kp[i] * error[i] + self.Kd[i] * self.error_change[i] + self.Ki[i] * self.error_sum[i])

        # Controlling roll, pitch, throttle and yaw using the PID outputs.
        self.drone_command.rcRoll = 1500 + pid_output[0]
        self.drone_command.rcPitch = 1500 + pid_output[1]
        self.drone_command.rcYaw = 1500 # angle = ((math.atan(out[1]/out[0])  180)/math.pi)  5.556 + 1000
        self.drone_command.rcThrottle = 1500 + pid_output[2]

        # Assigning current error to the previous error.
        self.prev_error = error

        # Publishing the drone_command
        self.cmd_pub.publish(self.drone_command)

        # retrun error
        return error


if __name__ == '__main__':

    i = 0 # counts the number of setpoints reached
    all_set_points = [[19, 72, 3], [19.0000451704, 72, 3], [19.0000451704, 72, 0.31]] # setpoints

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time.

    while not rospy.is_shutdown():
        try:
            # call pid function 
            error = e_drone.pid(all_set_points[i])

            # Check threshold box
            if abs(error[0]) < 0.000004517 and abs(error[1]) < 0.0000047487 and abs(error[2]) < 0.2:
                i += 1 # If the drone has reached a setpoint then move to next setpoints
            # If the drone has reached the last setpoint
            if i == 3: 
                e_drone.cmd_pub.publish(1500,1500,1500,1000,0,0,0,0) # stop the drone at final setpoint
                break
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass