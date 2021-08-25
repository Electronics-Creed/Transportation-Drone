#!/usr/bin/env python

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, LaserScan ,Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import cv2
import numpy as np
from std_msgs.msg import Float32, String
from vitarana_drone.srv import *
import rospy
import time
import tf
import math


# Class for QR code detection
class image_proc():

    # Initialise everything
    def __init__(self):
        self.bottom_distance = 0
        self.drone_gps_coordinates = [0,0,0]
        self.location = [0, 0, 0]

        self.if_destination = False

        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        rospy.Subscriber('/edrone/range_finder_bottom', LaserScan, self.drone_range_finder_bottom)

        self.img = np.empty([]) # This will contain your image frame from camera
        self.bridge = CvBridge()
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)

        # Callback function for drone_gps
    def drone_gps_callback(self, msg):
        self.drone_gps_coordinates[0] = msg.latitude
        self.drone_gps_coordinates[1] = msg.longitude

    # Callback function of camera topic
    def image_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            # Checking if the drone is at a suitable height to scan the QR code
            if 0.8 < self.bottom_distance < 1.2 and abs(self.drone_gps_coordinates[0] - e_drone.box_location[0]) < 0.000004 and abs(self.drone_gps_coordinates[1] - e_drone.box_location[1]) < 0.000004 and not self.if_destination:
                d = decode(self.img) # Decode the QR code
                try:
                    self.location = d[0].data.decode().split(",") # extract the gps co-ordinates
                    self.if_destination = True # if found the QR code change the value of this variable to True
                except IndexError:
                    pass
        except CvBridgeError as e:
            print(e)
            return

    # Range_finder_bottom callback function
    def drone_range_finder_bottom(self, msg):
        self.bottom_distance = msg.ranges[0]


# Class for position control, obstacle avoidance and navigation
class Edrone():
    """docstring for Edrone"""
    def __init__(self):

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

        # Kp, Kd and ki for [latitude, longitude, altitude].
        self.Kp = [250000, 250000, 220]
        self.Ki = [0.104 * 1e-3, 0.008 * 1e-3, 3.2]
        self.Kd = [429000, 450000, 220]
        
        # Other necessary variables
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]

        self.state = 0 # Store drone state
        self.j = 0 # Counter variable

        self.is_picked = False # To check if the box is picked

        self.height = 26 # To move the drone at fixed height
        self.all_next_setpoint = [] # To store the next setpoints

        self.obsracle_ranges = [0,0,0,0] # Contains range_finder_top sensor readings
        self.obstracle_move_to = [0,0,0] # If obstacle found then drone moves to this location

        self.box_location = [19.0007046575, 71.9998955286, 22] # Box location
        self.above_box_location = [self.box_location[0],self.box_location[1], self.height] # Coordinates above the box at fixed height
        self.drop_location = [0, 0, 0] # Drop location
        self.above_drop_location = [0, 0, 0] # Coordinates above the drop location at fixed height

        # This is the sample time needed to run pid.
        self.sample_time = 0.060  # in seconds

        # Publishing /drone_command and
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper) # Creating a service client

        # Subscribing to /edrone/gps, /edrone/range_finder_top and /edrone/gripper_check
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.drone_range_finder_top)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)
        rospy.wait_for_service('/edrone/activate_gripper') # Wait for service


    # Range_finder_top callback function
    def drone_range_finder_top(self, msg):
        self.obsracle_ranges = msg.ranges[:4]


    # Gripper_check callback function
    def gripper_check(self, msg):
        self.activate_gripper = msg.data


    # Callback function for drone_gps
    def drone_gps_callback(self, msg):
        self.drone_gps_coordinates[0] = msg.latitude
        self.drone_gps_coordinates[1] = msg.longitude
        self.drone_gps_coordinates[2] = msg.altitude
        print(self.drone_gps_coordinates, self.above_box_location)

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


    # Function to get required set_points
    def get_next_setpoint(self, desired_position):

        # Resetting variables
        self.all_next_setpoint = []
        self.j = 0

        desired_position_meters = (self.lat_to_x(desired_position[0]), self.long_to_y(desired_position[1])) # Getting desired position in meters

        distance = math.sqrt((desired_position_meters[0] - self.drone_gps_coordinates_meters[0]) ** 2 + (desired_position_meters[1] - self.drone_gps_coordinates_meters[1]) ** 2) # Calculating distance between desired position and current drone position
        d = int(math.floor(distance))

        number_of_setpoints = max(int(d / 10), 1) # Setpoints are spaced at 7 meters

        self.all_next_setpoint.append([self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], self.height])

        # Calculating all the next setpoints and storing in self.all_next_setpoint
        for i in range(0, d, d / number_of_setpoints):
            x = (i * desired_position_meters[0] + (d - i) * self.drone_gps_coordinates_meters[0]) / (d)
            y = (i * desired_position_meters[1] + (d - i) * self.drone_gps_coordinates_meters[1]) / (d)
            self.all_next_setpoint.append([self.x_to_lat(x), self.y_to_long(y), self.height])

        self.all_next_setpoint = self.all_next_setpoint[1:]
        self.all_next_setpoint.append([desired_position[0], desired_position[1], self.height])


    # Function to check and change the state if necessary 
    def modify_state(self, desired_position):
        # If drone has reached the desired position change the state to 1
        if abs(desired_position[0] - self.drone_gps_coordinates[0]) < 0.0000022585 and abs(desired_position[1] - self.drone_gps_coordinates[1]) < 0.0000022585: # if near destination
            self.state = 1
        # If drone has not reached the desired position change the state to 0
        else:
            self.state = 0

        # If drone detects an obstacle then change the state to 2 
        for i in self.obsracle_ranges:
            if 5 < i < 10:
                self.state = 2
                self.avoid_obstacle() # Call function to avoid obstacle
                break


    # Function to pick up or drop the box
    def pick_and_drop_box(self, desired_position, grip):

        # Check if the drone has reached the desired_position at some height
        if abs(desired_position[0] - self.drone_gps_coordinates[0]) < 0.0000022585 and abs(desired_position[1] - self.drone_gps_coordinates[1]) < 0.0000022585: # 
            rospy.loginfo('drone decending')
            if self.activate_gripper == str(grip): # Check if the gripper can be activated
                rospy.loginfo('box gripped')
                self.gripper(grip) # Send request to the server to grip the box
                self.is_picked = grip # Make variable True if gripped

                # Generate the next setpoints and make the state 0
                self.get_next_setpoint(self.above_drop_location)
                self.state = 0

            # If the drone can't grip the box
            else:
                self.pid(desired_position) # Move drone towards the box

                # If drone has scanned QR code then update the drop location 
                if image_proc_obj.if_destination:
                    x, y, z = [image_proc_obj.location[0], image_proc_obj.location[1], image_proc_obj.location[2]]
                    self.drop_location = [float(x.encode("utf-8")), float(y.encode("utf-8")), float(z.encode("utf-8"))]
                    self.above_drop_location = [self.drop_location[0], self.drop_location[1], self.height]

            # If the drone has reached the drop location
            if not grip and abs(self.drone_gps_coordinates[2] - self.drop_location[2]) < 0.3:
                    print('box dropped')
                    self.gripper(grip) # Detach the box
                    self.cmd_pub.publish(1500, 1500, 1500, 1000, 0, 0, 0, 0) # Stop the propellers
                    exit(0) # Exit program

        # If the drone has not reached the desired_position at some height
        else:
            self.all_next_setpoint = [self.above_box_location]
            self.j = 0
            self.state = 0


    # Funtion to avoid obstacle
    def avoid_obstacle(self):
        max_dist = 15
        min_dist = 0.5
        distance = 7
        near_wall = 0.5

        # If the obstacle is found towards left
        if min_dist < self.obsracle_ranges[3] < max_dist and self.obsracle_ranges[0] > max_dist and self.obsracle_ranges[2] > max_dist:
            rospy.loginfo("obstacle to the left")
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] + (max_dist - self.obsracle_ranges[3] - near_wall)) , self.y_to_long(self.drone_gps_coordinates_meters[1] - distance), self.height

        # If the obstacle is found towards front
        elif self.obsracle_ranges[3] > max_dist and min_dist < self.obsracle_ranges[0] < max_dist and self.obsracle_ranges[2] > max_dist:
            rospy.loginfo("obstacle to the front")
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] - distance) , self.y_to_long(self.drone_gps_coordinates_meters[1] - (max_dist - self.obsracle_ranges[0] - near_wall)), self.height
        
        # If the obstacle is found towards back
        elif self.obsracle_ranges[3] > max_dist and self.obsracle_ranges[0] > max_dist and min_dist < self.obsracle_ranges[2] < max_dist:
            rospy.loginfo("obstacle to the back")
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] - distance) , self.y_to_long(self.drone_gps_coordinates_meters[1] + (max_dist - self.obsracle_ranges[2] - near_wall)), self.height

        # If the obstacle is found towards left and front
        elif min_dist < self.obsracle_ranges[3] < max_dist and min_dist < self.obsracle_ranges[0] < max_dist and self.obsracle_ranges[2] > max_dist:
            rospy.loginfo("obstacle to the left and front")
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] + (max_dist - self.obsracle_ranges[3] - near_wall)) , self.y_to_long(self.drone_gps_coordinates_meters[1] - (max_dist - self.obsracle_ranges[0] + near_wall)), self.height

        # If the obstacle is found towards left and back
        elif min_dist < self.obsracle_ranges[3] < max_dist and self.obsracle_ranges[0] > max_dist and min_dist < self.obsracle_ranges[2] < max_dist:
            rospy.loginfo("obstacle to the left and back")
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] + (max_dist - self.obsracle_ranges[3] - near_wall)) , self.y_to_long(self.drone_gps_coordinates_meters[1] + (max_dist - self.obsracle_ranges[2] + near_wall)), self.height

        # If the obstacle is found towards front and back
        elif self.obsracle_ranges[3] > max_dist and min_dist < self.obsracle_ranges[0] < max_dist and min_dist < self.obsracle_ranges[2] < max_dist:
            rospy.loginfo("obstacle to the front and back")
            middle = (self.obsracle_ranges[0] + self.obsracle_ranges[2]) / 2
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] - distance) , self.y_to_long(self.drone_gps_coordinates_meters[1] - (middle - self.obsracle_ranges[0])), self.height

        # If the obstacle is found towards left, front and right
        elif min_dist < self.obsracle_ranges[3] < max_dist and min_dist < self.obsracle_ranges[0] < max_dist and min_dist < self.obsracle_ranges[2] < max_dist:
            rospy.loginfo("obstacle to the left, front and right") 
            self.obstracle_move_to = self.x_to_lat(self.drone_gps_coordinates_meters[0] - distance) , self.y_to_long(self.drone_gps_coordinates_meters[1] - (max_dist - self.obsracle_ranges[0] + near_wall)), self.height

        # If no obstacle is found
        elif self.obsracle_ranges[3] > max_dist and self.obsracle_ranges[1] > max_dist and self.obsracle_ranges[0] > max_dist:
            rospy.loginfo("no obstracle is found")
            if self.is_picked:
                self.get_next_setpoint(self.above_drop_location) # Check for drop location
            else:
                self.get_next_setpoint(self.above_box_location) # Check for pick location
            self.state = 0 # Change the state to 0

        # If the obstacle is unknown or too near
        # else:
        #     rospy.loginfo("unknown type of obstacle or obstacle too near")
        #     if self.is_picked:
        #         self.get_next_setpoint(self.above_drop_location) # Check for drop location
        #     else:
        #         self.get_next_setpoint(self.above_box_location) # Check for pick location

        #     self.state = 0 # Change the state to 0




    # PID finction
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
        self.drone_command.rcYaw = 1500
        self.drone_command.rcThrottle = 1500 + pid_output[2]

        # Assigning current error to the previous error.
        self.prev_error = error

        # Publishing the drone_command
        self.cmd_pub.publish(self.drone_command)

        # If desired setpoint is reached
        if abs(setpoint_gps_coordinates[0] - self.drone_gps_coordinates[0]) < 0.0000022585 and abs(setpoint_gps_coordinates[1] - self.drone_gps_coordinates[1]) < 0.0000022585 and abs(setpoint_gps_coordinates[2] - self.drone_gps_coordinates[2]) < 0.2:
            # If state is 2 the check for obstacles again
            if self.state == 2:
                self.avoid_obstacle()
            # Go to next setpoint
            else:
                self.j += 1


def main():
    # If state is 0
    if e_drone.state == 0:
	    if not e_drone.is_picked: # If box has not been picked
		e_drone.pid(e_drone.all_next_setpoint[e_drone.j]) # Call PID function
		e_drone.modify_state(e_drone.above_box_location) # Call modify_state
	    else: # If box has been picked
		e_drone.pid(e_drone.all_next_setpoint[e_drone.j]) # Call PID function
		e_drone.modify_state(e_drone.above_drop_location) # Call modify_state     

    # If state is 1
    elif e_drone.state == 1:
        if not e_drone.is_picked: # If box has not been picked
            e_drone.pick_and_drop_box(e_drone.box_location, True) # Call pick_and_drop_box function
        else: # If box has been picked
            e_drone.pick_and_drop_box(e_drone.drop_location, False) # Call pick_and_drop_box function

    # If state is 2
    elif e_drone.state == 2:
        e_drone.pid(e_drone.obstracle_move_to) # Call PID function
        
    else:
        rospy.loginfo('unknown state')

        

if __name__ == '__main__':

    e_drone = Edrone() # Edrone object creation
    image_proc_obj = image_proc() # image_proc object creation
    r = rospy.Rate(1/e_drone.sample_time) 

    e_drone.get_next_setpoint(e_drone.above_box_location) # Generate setpoints to move towards box location

    while not rospy.is_shutdown():
        try:
            main()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
