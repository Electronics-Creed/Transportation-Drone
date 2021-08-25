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
import cv2
import csv


# Class for position control, obstacle avoidance and navigation
class Edrone():
    """docstring for Edrone"""
    def __init__(self):

        rospy.init_node('position_controller')  # initializing ros node with name 'position_controller'

        # This corresponds to gps coordinates of eDrone
        self.drone_gps_coordinates = [0,0,0]

        # Declaring drone_command of message type edrone_cmd and initializing values
        self.drone_command = edrone_cmd()
        self.drone_command.rcRoll = 1500.0
        self.drone_command.rcPitch = 1500.0
        self.drone_command.rcYaw = 1500.0
        self.drone_command.rcThrottle = 1000.0

        # # Kp, Kd and ki for [lat, long, thr]
        self.Kp = [2.2, 2.2, 150]
        self.Ki = [0.000045, 0.000045, 0.003]
        self.Kd = [46, 46, 3333]

        # Other necessary variables
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]

        self.state = 0 # Store drone state
        self.j = 0 # Counter variable
        self.i = 0 # counter when moving between different destinations

        self.is_box_picked = False # To check if the box is picked
        self.obstacle_direction = 0 # direction of obstacle
        self.obstacle_found = False # check if obstacle found 

        self.all_next_setpoint = [] # To store the next setpoints

        self.marker_location = [0, 0, 0] # exact maarker location
        self.marker = [0,0] # marker distance 
        self.landing_found = False # check if landing is found

        self.obsracle_ranges = [0,0,0,0] # Contains range_finder_top sensor readings
        self.obstracle_move_to = [0,0,0] # If obstacle found then drone moves to this location

        self.target = (0, 0, 0, 0) # Holds the values x, y, w and h of the image
        self.logo_cascade = cv2.CascadeClassifier('/home/user/catkin_ws/src/vitarana_drone/data/cascade.xml') # Invoking the cascade classifier
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)

        self.bridge = CvBridge()

        self.logo = [] # store all the bounding box dimensions

        self.above_drop_location = [] # Coordinates above the building coordinates
        self.drop_location = [] # building locations
        with open('/home/user/catkin_ws/src/vitarana_drone/scripts/manifest.csv') as csv_file: # read csv file
		    csv_reader = csv.reader(csv_file, delimiter=',')

		    for ind, row in enumerate(csv_reader): # read all the rows
    		    	self.drop_location.append((self.lat_to_x(float(row[1])), self.long_to_y(float(row[2])), float(row[3])))
    			self.above_drop_location.append((self.drop_location[ind][0], self.drop_location[ind][1], self.drop_location[ind][2] + 18)) # Coordinates above the building location 

        self.sample_time = 0.060  # This is the sample time needed to run pid.

        # Publishing /drone_command and
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper) # Creating a service client

        # Subscribing to /edrone/gps, /edrone/range_finder_top and /edrone/gripper_check
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.drone_range_finder_top)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)
        rospy.wait_for_service('/edrone/activate_gripper') # Wait for service

        self.box_location = [(self.lat_to_x(18.9999864489), self. long_to_y(71.9999430161), 8.44099749139)] # Box location

        self.box_location.append((self.box_location[0][0] + 3 ,self.box_location[0][1],self.box_location[0][2])) # append all the box locations
        self.box_location.append((self.box_location[0][0] + 1.5, self.box_location[0][1] - 1.5, self.box_location[0][2]))
        self.box_location.append((0,0,self.box_location[0][2]))
        
        self.above_box_location = []
        for x,y,z in self.box_location: # calculate above box locations
            self.above_box_location.append((x,y,z+18))


    # Range_finder_top callback function
    def drone_range_finder_top(self, msg):
        self.obsracle_ranges = msg.ranges[:4]


    # Gripper_check callback function
    def gripper_check(self, msg):
        self.activate_gripper = msg.data


    # Callback function for drone_gps
    def drone_gps_callback(self, msg):
        self.drone_gps_coordinates[0] = self.lat_to_x(msg.latitude)
        self.drone_gps_coordinates[1] = self.long_to_y(msg.longitude)
        self.drone_gps_coordinates[2] = msg.altitude


    def image_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # converting to gray image
            self.logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05) # get all the x,y,w,h values of all the marker

            for i in range(len(self.logo)):
                x, y, w, h = self.logo[i] # unpacking self.logo
                area = w * h # calculating area of each bounding box
                if area < 950 and area != 0: # If area of the box detected is less than 1220
                    # making the center of image (0, 0)
                    x -= 200
                    y -= 200
                    self.target = x, y, w, h # saving this value logo into target
                    break

        except CvBridgeError as e:
            print(e)
            return


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
        horizontal_dist = 38 # distance between horizontal setpoints
        vertical_dist = 15 # distance between vertical setpoints


        vertical_distance = desired_position[2] - self.drone_gps_coordinates[2] # vertical distance to be moved
        vertical_number_of_setpoints = vertical_distance // vertical_dist # number of vertical setpoints

        for i in range(int(vertical_number_of_setpoints)): # calculate all the vertical setpoints
            self.all_next_setpoint.append((self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], self.drone_gps_coordinates[2] + vertical_dist)) 
        self.all_next_setpoint.append((self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], desired_position[2]))

        total_distance = math.sqrt((desired_position[0] - self.drone_gps_coordinates[0]) ** 2 + (desired_position[1] - self.drone_gps_coordinates[1]) ** 2) # Calculating distance between desired position and current drone position
        horizontal_number_of_setpoints = total_distance // horizontal_dist # number of horizontal setpoints

        if horizontal_number_of_setpoints > 0:
            p_vector = desired_position[0] - self.drone_gps_coordinates[0], desired_position[1] - self.drone_gps_coordinates[1] # Vector from drone position to the desired position
            p_mag = 1 / float(horizontal_number_of_setpoints) # Number of partitions of the p_vector

            next_point = self.drone_gps_coordinates

            for i in range(int(horizontal_number_of_setpoints)):# calculate horizontal setponints
                next_point[0] = next_point[0] + p_mag * p_vector[0]
                next_point[1] = next_point[1] + p_mag * p_vector[1]
                self.all_next_setpoint.append((next_point[0],next_point[1], desired_position[2])) # append the next setpoint
        self.all_next_setpoint.append(desired_position)


    # Function to check and change the state if necessary
    def modify_state(self, desired_position):
        # If drone has reached the desired position
		if abs(desired_position[0] - self.drone_gps_coordinates[0]) < 0.2 and abs(desired_position[1] - self.drone_gps_coordinates[1]) < 0.2 and abs(desired_position[2] - self.drone_gps_coordinates[2]) < 0.2: # if near destination
			if not self.is_box_picked: # check box is picked 
				self.state = 1

			elif self.is_box_picked and not self.landing_found: # check landing marker is found
				self.landing_found = True
				self.state = 3
				self.get_land_location() # get landing marker location

			if self.state == 3: # check id state is 3
			    	self.get_land_location() # generate marker location
				if abs(self.marker_location[0] - self.drone_gps_coordinates[0]) < 0.8 and abs(self.marker_location[1] - self.drone_gps_coordinates[1]) < 0.8: # verify marker location
				    self.state = 4
	    			    self.marker_location[2] = self.drop_location[self.i][2] # change the height of marker location
	    # check if state is 3
		elif self.state == 3:
			pass
		# check state is 4
		elif self.state == 4:
		    if abs(self.marker_location[0] - self.drone_gps_coordinates[0]) < 0.2 and abs(self.marker_location[1] - self.drone_gps_coordinates[1]) < 0.2 and abs(self.marker_location[2] - self.drone_gps_coordinates[2]) < 0.6: # check marker location is reached
			    self.gripper(False)# drop box 

			    self.state = 0 # change the state to 0
			    self.i += 1
			    self.is_box_picked = False
                	    self.landing_found = False
			    self.get_next_setpoint(self.above_box_location[self.i]) # Generate setpoints to get next box
		# if no condition is true change the state to 0
		else:
	    		self.state = 0
	    		
            	if self.state != 1 or self.state != 4:# check if state is not 1 or 4
    	            for ind, obs in enumerate(self.obsracle_ranges): # check for obstacles
    			if 5 < obs < 10: # check if obstacle found
    			    self.state = 2
    			    self.obstacle_direction = ind + 1
    			    self.avoid_obstacle() # Call function to avoid obstacle
    			    self.obstacle_found = True
    		            self.j = 0
    			    break

    # Based on the direction obstacle found get the coordinate to move
    def after_obtacle_avoided_move_to(self): 
        if self.obstacle_direction == 1:
            return [self.drone_gps_coordinates[0] - 8, self.drone_gps_coordinates[1] + 15, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 2:
            return [self.drone_gps_coordinates[0] + 15, self.drone_gps_coordinates[1] + 8, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 3:
            return [self.drone_gps_coordinates[0] + 8, self.drone_gps_coordinates[1] - 15, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 4:
            return [self.drone_gps_coordinates[0] - 15, self.drone_gps_coordinates[1] - 8, self.drone_gps_coordinates[2]]


    # get the precise location of the landing marker
    def get_land_location(self):
        hfov_rad = 1.3962634
        img_width = 400

        x, y, w, h = self.target # get marker dimensions

        focal_length = (img_width/2)/math.tan(hfov_rad/2) # calculate focal length

        # Calculating distance between drone and target (only magnitude)
        self.marker[0] = abs((x + w / 2)*(self.drone_gps_coordinates[2] - self.drop_location[self.i][2]) / focal_length)
        self.marker[1] = abs((y + h / 2)*(self.drone_gps_coordinates[2] - self.drop_location[self.i][2]) / focal_length)

        # Necessary sign allocation for marker
        if x > 0 and y > 0:
            self.marker[0] = self.marker[0]
            self.marker[1] = -self.marker[1]
        elif x < 0 and y > 0:
            self.marker[0] = -self.marker[0]
            self.marker[1] = -self.marker[1]
        elif x < 0 and y < 0:
            self.marker[0] = -self.marker[0]
            self.marker[1] = self.marker[1]
        else:
            self.marker[0] = self.marker[0]
            self.marker[1] = self.marker[1]

        # calculating global marker location
        global_marker_X = self.marker[0] + self.drone_gps_coordinates[0]
        global_marker_Y = self.marker[1] + self.drone_gps_coordinates[1]

        self.marker_location = [global_marker_X, global_marker_Y, self.drone_gps_coordinates[2]] # target location


    def pick_box(self):
    	if self.activate_gripper == str(True): # Check if the gripper can be activated
            rospy.loginfo('box gripped')
            self.gripper(True) # Send request to the server to grip the box
            self.is_box_picked = True # Make variable True if gripped

            # Generate the next setpoints and make the state 0
            self.get_next_setpoint(self.above_drop_location[self.i])
            self.state = 0

        # If the drone can't grip the box
        else:
            rospy.loginfo('drone decending')
            self.pid(self.box_location[self.i]) # Move drone towards the box


    # Funtion to avoid obstacle
    def avoid_obstacle(self):
        max_dist = 10 
        min_dist = 0.5
        distance = 9
        near_wall = 2

        # If the obstacle is found towards left
        if min_dist < self.obsracle_ranges[3] < max_dist:
            rospy.loginfo("obstacle to the left")
            obstracle_move_to = (self.drone_gps_coordinates[0] + (max_dist - self.obsracle_ranges[3] - near_wall)) , (self.drone_gps_coordinates[1] - distance), self.drone_gps_coordinates[2]

        # If the obstacle is found towards front
        elif min_dist < self.obsracle_ranges[0] < max_dist:
            rospy.loginfo("obstacle to the front")
            obstracle_move_to = (self.drone_gps_coordinates[0] - distance) , (self.drone_gps_coordinates[1] - (max_dist - self.obsracle_ranges[0] - near_wall)), self.drone_gps_coordinates[2]

        # If the obstacle is found towards back
        elif min_dist < self.obsracle_ranges[2] < max_dist:
            rospy.loginfo("obstacle to the back")
            obstracle_move_to = (self.drone_gps_coordinates[0] + distance) , (self.drone_gps_coordinates[1] + (max_dist - self.obsracle_ranges[2] - near_wall)), self.drone_gps_coordinates[2]
        
        elif min_dist < self.obsracle_ranges[1] < max_dist:
            rospy.loginfo("obstacle to the right")
            obstracle_move_to = (self.drone_gps_coordinates[0] - (max_dist - self.obsracle_ranges[1] - near_wall)) , (self.drone_gps_coordinates[1] + distance), self.drone_gps_coordinates[2]

        # If no obstacle is found
        elif self.obsracle_ranges[3] > max_dist and self.obsracle_ranges[1] > max_dist and self.obsracle_ranges[0] > max_dist:
            rospy.loginfo("no obstracle is found")
            if self.is_box_picked:
                self.get_next_setpoint(self.above_drop_location[self.i]) # Check for drop location
            else:
                self.get_next_setpoint(self.above_box_location[self.i]) # Check for pick location
            self.state = 0 # Change the state to 0
        else:
            if self.is_box_picked:
                self.get_next_setpoint(self.above_drop_location[self.i]) # Check for drop location
            else:
                self.get_next_setpoint(self.above_box_location[self.i]) # Check for pick location
            self.state = 0 # Change the state to 0

        self.obstracle_move_to = [obstracle_move_to, self.after_obtacle_avoided_move_to()] # calculate setpoints to avoid obstacle

    def check_dot_product(self):

        if self.obstacle_direction == 1: # based on obstacle direction get u vector
            u_vector = -3, 0
        if self.obstacle_direction == 2:
            u_vector = 0, 3
        if self.obstacle_direction == 3:
            u_vector = 3, 0
        if self.obstacle_direction == 4:
            u_vector = 0, -3

        if self.is_box_picked: # based on box is picked or not get v vector
            v_vector = self.above_drop_location[self.i][0] - self.drone_gps_coordinates[0], self.above_drop_location[self.i][1] - self.drone_gps_coordinates[1]  # Check for drop location
        else:
            v_vector = self.above_box_location[self.i][0] - self.drone_gps_coordinates[0], self.above_box_location[self.i][1] - self.drone_gps_coordinates[1] # Check for pick location
        dot_p = u_vector[0] * v_vector[0] + u_vector[1] * v_vector[1] # calculate the dot product of u and v
        
        if dot_p < 0: # check dot product is positive or negative
            return False # Continue to follow wall
        else:
            return True # get to the destination point acoordingly ( drop or box depends )

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
            pid_output[i] = self.Kp[i] * error[i] + self.Kd[i] * self.error_change[i] + self.Ki[i] * self.error_sum[i]

        # Controlling roll, pitch, throttle and yaw using the PID outputs.
        self.drone_command.rcRoll = 1500 + pid_output[0]
        self.drone_command.rcPitch = 1500 - pid_output[1]
        self.drone_command.rcYaw = 1500
        self.drone_command.rcThrottle = 1500 + pid_output[2]

        # Assigning current error to the previous error.
        self.prev_error = error

        # Publishing the drone_command
        self.cmd_pub.publish(self.drone_command)

        # If desired setpoint is reached
        if abs(setpoint_gps_coordinates[0] - self.drone_gps_coordinates[0]) < 0.19 and abs(setpoint_gps_coordinates[1] - self.drone_gps_coordinates[1]) < 0.19 and abs(setpoint_gps_coordinates[2] - self.drone_gps_coordinates[2]) < 0.19:
            self.j += 1 # call modify state
            if self.state == 2 and self.j == 1: # if state is 2 and j is 1
                for ind, obs in enumerate(self.obsracle_ranges): # check if obstacle is still present
		        if 1 < obs < 10:
		            self.state = 2
		            self.obstacle_direction = ind + 1
		            self.avoid_obstacle() # Call function to avoid obstacle
		            self.obstacle_found = True
		            self.j = 0
		            break
		        else: # if no obstacle is found
		            self.obstacle_found = False
	        if not self.obstacle_found: # if obstacle is not found 
	            if self.check_dot_product(): # check the dot product
	                self.state = 0 # change the state to 0
	                if self.is_box_picked:
	                    self.get_next_setpoint(self.above_drop_location[self.i])
	                else:
	                    self.get_next_setpoint(self.above_box_location[self.i])

            if self.state == 2 and self.j == 2: # if the obstacle is completly avoied 
                self.state = 0
                if self.is_box_picked:
                    self.get_next_setpoint(self.above_drop_location[self.i])
                else:
                    self.get_next_setpoint(self.above_box_location[self.i])

def main():
    if e_drone.state == 0:
    	if not e_drone.is_box_picked: # If box has not been picked
    		e_drone.modify_state(e_drone.above_box_location[e_drone.i])
    		e_drone.pid(e_drone.all_next_setpoint[e_drone.j])
    	else:
    		e_drone.modify_state(e_drone.above_drop_location[e_drone.i])
    		e_drone.pid(e_drone.all_next_setpoint[e_drone.j])

    elif e_drone.state == 1: # pick up the box
    	if e_drone.i == 3: # check if all the boxes have been delivered
        	e_drone.pid([0,0,e_drone.box_location[0][2]])
        	print(e_drone.box_location[0])
        	if abs(e_drone.drone_gps_coordinates[2] - e_drone.box_location[0][2] + 1) < 0.2: # check if drone has come to the starting point
        		e_drone.cmd_pub.publish(1500,1500,1500,1000,0,0,0,0) # stop the propellers
        		exit(0)
        else:
        	e_drone.pick_box() # Call pick_and_drop_box function

    elif e_drone.state == 2:# avoid obstacle
    	e_drone.pid(e_drone.obstracle_move_to[e_drone.j])

    elif e_drone.state == 3: # get marker location
        e_drone.pid(e_drone.marker_location) # call pid
        e_drone.modify_state(e_drone.marker_location)

    elif e_drone.state == 4: # land and drop box
        e_drone.pid(e_drone.marker_location) # call pid and modify state
        e_drone.modify_state(e_drone.marker_location)

    else:
        rospy.loginfo('unknown state')


if __name__ == '__main__':

    e_drone = Edrone() # Edrone object creation
    r = rospy.Rate(1/e_drone.sample_time)

    e_drone.get_next_setpoint(e_drone.above_box_location[e_drone.i]) # Generate setpoints to move towards box location

    while not rospy.is_shutdown():
        try:

            main()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
