#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf


# Class for attitude control
class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('attitude_controller')  # initializing ros node with name attitude_controller 

        # This corresponds to current orientation of eDrone in quaternion format. This value will be updated each time in imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [1500.0, 1500.0, 1500.0, 1000.0]

        # The setpoint of orientation in euler angles at which the drone will be stabilized.
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # Kp, Kd and ki for [roll, pitch, yaw]. 
        self.Kp = [21.18, 21.18, 406.08]
        self.Ki = [0.000000024, 0.000000024, 0.00288 * 4]
        self.Kd = [835, 835, 1495]
        
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]

        # # This is the sample time needed to run pid.
        self.sample_time = 0.060  

        # Publishing /edrone/pwm
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)

        # Subscribing to /drone_command, /edrone/imu/data
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)

    # Imu callback function
    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w

    # drone_command callback function
    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw
        self.setpoint_cmd[3] = msg.rcThrottle

    # PID finction
    def pid(self):

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[1], self.drone_orientation_euler[0], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
        throttle = (self.setpoint_cmd[3] - 1000) * 1.024

        # Create variables to store errors and pid output.
        error = [0,0,0]
        pid_output = [0, 0, 0]

        for i in range(len(error)):

            # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis.
            self.setpoint_euler[i] = self.setpoint_cmd[i] * 0.02 - 30

            # Calculating errors in the roll, pitch and yaw axis.
            error[i] = (self.setpoint_euler[i] - self.drone_orientation_euler[i])

            # Calculating error_sum for the integral part of PID equation.
            self.error_sum[i] += error[i]

            # Calculating error_change for the differential part of PID equation.
            self.error_change[i] = error[i] - self.prev_error[i]

            # Calculating PID output of the three axes. 
            pid_output[i] = self.Kp[i] * error[i] + self.Kd[i] * self.error_change[i] + self.Ki[i] * self.error_sum[i]

        # Controlling propeller speed using the PID outputs.
        self.pwm_cmd.prop1 = throttle - pid_output[0] + pid_output[1] - pid_output[2]
        self.pwm_cmd.prop2 = throttle - pid_output[0] - pid_output[1] + pid_output[2]
        self.pwm_cmd.prop3 = throttle + pid_output[0] - pid_output[1] - pid_output[2]
        self.pwm_cmd.prop4 = throttle + pid_output[0] + pid_output[1] + pid_output[2]

        # Assigning current error to the previous error.
        self.prev_error = error

        # Limiting the output value between 0 and 1024
        if self.pwm_cmd.prop1 > self.max_values[0]:
            self.pwm_cmd.prop1 = self.max_values[0]
        if self.pwm_cmd.prop2 > self.max_values[1]:
            self.pwm_cmd.prop2 = self.max_values[1]
        if self.pwm_cmd.prop3 > self.max_values[2]:
            self.pwm_cmd.prop3 = self.max_values[2]
        if self.pwm_cmd.prop4 > self.max_values[3]:
            self.pwm_cmd.prop4 = self.max_values[3]

        if self.pwm_cmd.prop1 < self.min_values[0]:
            self.pwm_cmd.prop1 = self.min_values[0]
        if self.pwm_cmd.prop2 < self.min_values[1]:
            self.pwm_cmd.prop2 = self.min_values[1]
        if self.pwm_cmd.prop3 < self.min_values[2]:
            self.pwm_cmd.prop3 = self.min_values[2]
        if self.pwm_cmd.prop4 < self.min_values[3]:
            self.pwm_cmd.prop4 = self.min_values[3]

        # Publishing the propeller speeds
        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(1/e_drone.sample_time)  # specify rate in Hz based upon your desired PID sampling time.
    while not rospy.is_shutdown():
        try:
            e_drone.pid()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass