#!/usr/bin/env python


# Importing the required libraries

from vitarana_drone.msg import *
from sensor_msgs.msg import NavSatFix, LaserScan ,Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float32, String
from vitarana_drone.srv import *
import rospy
import time
import tf
import math
import csv
import os

PATH = os.path.dirname(os.path.abspath(__file__)).replace('/scripts',"") # path to dir


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
        self.Kp = [[2.8, 2.8, 250], [3, 3, 400], [2.5, 2.5, 200], [1.2, 1.2, 160], [2.7, 2.7, 200]] 
        self.Ki = [[0.00005, 0.00005, 0.006], [0.000005, 0.000005, 0.0015], [0.000045, 0.000045, 0.006], [0.000045, 0.000045, 0.006], [0.000045, 0.000045, 0.006]]
        self.Kd = [[80, 80, 4500], [60, 60, 5000], [75, 75, 4500], [40, 40, 4000], [70, 70, 4000]]

        # Publishing /drone_command and
        self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
        self.gripper = rospy.ServiceProxy('/edrone/activate_gripper', Gripper) # Creating a service client

        # Subscribing to /edrone/gps, /edrone/range_finder_top and /edrone/gripper_check
        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.drone_range_finder_top)
        rospy.Subscriber('/edrone/gripper_check', String, self.gripper_check)
        rospy.wait_for_service('/edrone/activate_gripper') # Wait for service

        # PID function variables
        self.error_sum = [0, 0, 0]
        self.error_change = [0, 0, 0]
        self.prev_error = [0, 0, 0]

        # Basic variables
        self.state = 0 # Store drone state
        self.j = 0 # Counter variable
        self.is_box_picked = False # To check if the box is picked
        self.is_delivered = False

        # Obstacle related variables
        self.obstacle_direction = 0 # direction of obstacle
        self.obstacle_found = False # check if obstacle found 
        self.obsracle_ranges = [0,0,0,0] # Contains range_finder_top sensor readings
        self.obstracle_move_to = [0,0,0] # If obstacle found then drone moves to this location

        # get_next_setpoint related variables
        self.all_next_setpoint = [] # To store the next setpoints

        # get_land location related variables
        self.marker_location = [0, 0, 0] # exact maarker location
        self.marker = [0,0] # marker distance 
        self.landing_found = False # check if landing is found
        self.heading = True 

        # image callback related variables
        self.target = (0, 0, 0, 0) # Holds the values x, y, w and h of the image
        self.logo_cascade = cv2.CascadeClassifier(PATH+'/data/cascade.xml') # Invoking the cascade classifier
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.logo = [] # store all the bounding box dimensions

        # Locations
        self.dict_abc = {} 
        self.dict_xyz = {}
        self.dict_location_to_abc = {}
        self.dict_location_to_xyz = {}
        self.location = []
        self.get_locations()
        self.box_location = [] # Box location
        self.above_box_location = []
        self.above_drop_location = [] # Coordinates above the building coordinates
        self.drop_location = [] # building locations
        
        self.sample_time = 0.060  # This is the sample time needed to run pid.


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


    # image callback function
    def image_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # converting to gray image
            self.logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05) # get all the x,y,w,h values of all the marker

            for i in range(len(self.logo)):
                x, y, w, h = self.logo[i] # unpacking self.logo
                area = w * h # calculating area of each bounding box
                # print(area)
                if 650 < area < 750 and area != 0: # If area of the box detected is less than 1220
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


    # calculate distance function
    def cal_distance(self,x,y):
        return math.sqrt((x[0]- y[0])**2 + (y[1] - x[1])**2)


    # function to write csv file
    def write_csv_file(self):

        fields = ['Type', 'Origin', 'Destination'] # heading
        if self.is_delivered: # check delivery
            typ = 'Delivery'
            origin = self.dict_location_to_abc[self.box_location]
            destination = str(self.x_to_lat(self.drop_location[0])) + ',' + str(self.y_to_long(self.drop_location[1])) + ',' + str(self.drop_location[2])
        else: # check for return
            typ = 'Return'
            origin = str(self.x_to_lat(self.box_location[0])) + ',' + str(self.y_to_long(self.box_location[1])) + ',' + str(self.box_location[2])
            destination = self.dict_location_to_xyz[self.drop_location]
            
        # data rows of csv file
        rows = []
        rows.append([typ, origin, destination]) # append to the list
        # name of csv file 
        filename = "/sequenced_manifest.csv"
            
        # writing to csv file 
        with open(PATH + filename, 'a') as csvfile:
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 

            # writing the fields
            if self.heading:
                self.heading = False
                csvwriter.writerow(fields) 
                
            # writing the data rows
            csvwriter.writerows(rows)


    # store all the marker and box location
    def get_locations(self):
        self.dict_abc = {'A1' : (self.lat_to_x(18.9998102845), self.long_to_y(72.000142461), 16.757981)} # to store all the grid locations
        self.dict_xyz = {'X1' : (self.lat_to_x(18.9999367615), self.long_to_y(72.000142461), 16.757981)}
        for i in range(3): # calculate the other grid locations
            for j in range(3):
                if not (i == 0 and j == 0):
                    self.dict_abc[chr(ord('A') + i) + str(j + 1)] = (self.dict_abc['A1'][0] + 1.5*i, self.dict_abc['A1'][1] - 1.5*j, self.dict_abc['A1'][2])
                    self.dict_xyz[chr(ord('X') + i) + str(j + 1)] = (self.dict_xyz['X1'][0] + 1.5*i, self.dict_xyz['X1'][1] - 1.5*j, self.dict_xyz['X1'][2])

        with open(PATH+'/scripts/manifest.csv') as csv_file: # read csv file
            csv_reader = csv.reader(csv_file, delimiter=',')

            for ind, row in enumerate(csv_reader): # read all the rows

                if  23 > ind > 4:
                    if row[0] == "DELIVERY": # if delivery unpack
                        q = True 
                        p = self.dict_abc[row[1]]
                        r = [float(i) for i in row[2].split(';')]
                        r = (self.lat_to_x(r[0]), self.long_to_y(r[1]), r[2])
                    elif row[0] == "RETURN ": # if return unpack
                        q = False
                        r = self.dict_xyz[row[-1].replace(' ', '')]
                        p = [float(i) for i in row[1].split(';')]
                        p = (self.lat_to_x(p[0]), self.long_to_y(p[1]), p[2])
                    self.location.append((q, p, r)) # store the locations
	self.box_marker_distance = {} 
	self.all_marker_locations = [] 
	self.all_box_locations = []
	for _,i,j in self.location: # store distances between boxes and markers
		self.box_marker_distance[i] = self.cal_distance(i,j)
		self.all_marker_locations.append(j)
		self.all_box_locations.append(i)

        self.dict_location_to_abc = { v:k for (k,v) in zip(self.dict_abc.keys(), self.dict_abc.values())}
        self.dict_location_to_xyz = { v:k for (k,v) in zip(self.dict_xyz.keys(), self.dict_xyz.values())}


    # function to calculate the minimum distance
    def get_minimum_distance(self):
        	distances = []
		for i in range(len(self.all_marker_locations)): # calculate the distances to delivery one box completely
			dist1 = self.cal_distance(self.drone_gps_coordinates, self.all_box_locations[i])
			dist2 = self.box_marker_distance[self.all_box_locations[i]]
			distances.append(dist1 + dist2)
		m = distances.index(min(distances)) # get the minimum of the distaces
        # update the variables
        	self.is_delivered = self.location[m][0] 
		self.drop_location = self.all_marker_locations[m]
		self.above_drop_location = (self.drop_location[0], self.drop_location[1], self.drop_location[2] + 22)
		self.box_location = self.all_box_locations[m]
		self.above_box_location = (self.box_location[0], self.box_location[1], self.box_location[2] + 3)
		self.all_marker_locations.remove(self.all_marker_locations[m])
		self.all_box_locations.remove(self.all_box_locations[m])
		self.location.remove(self.location[m])


    # Calculates setpoints based on drone position and desired position
    def get_next_setpoint(self, desired_position):
        # Resetting variables
        self.all_next_setpoint = [] # Contains all the generated setpoints
        self.j = 0
        horizontal_dist = 22 # distance between horizontal setpoints
        vertical_dist = 11 # distance between vertical setpoints

        if not self.is_box_picked:
            height = self.drone_gps_coordinates[2] + 3
            self.all_next_setpoint.append((self.drone_gps_coordinates[0], self.drone_gps_coordinates[1], height))
        else:
            height = desired_position[2]

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
                self.all_next_setpoint.append((next_point[0],next_point[1], height)) # append the next setpoint
        self.all_next_setpoint.append(desired_position)


    # Function to check and change the state if necessary
    def modify_state(self, desired_position):
        	# check state is 4
		if self.state == 4:
		    if abs(self.marker_location[0] - self.drone_gps_coordinates[0]) < 0.21 and abs(self.marker_location[1] - self.drone_gps_coordinates[1]) < 0.21 and abs(self.marker_location[2] - self.drone_gps_coordinates[2]) < 0.2: # check marker location is reached
                            self.gripper(False)# drop box 
                            self.write_csv_file()
    		            self.state = 0 # change the state to 0
    		            self.is_box_picked = False
                            self.landing_found = False
                            self.get_minimum_distance()
                    	    self.get_next_setpoint(self.above_box_location) # Generate setpoints to get next box
        # If drone has reached the desired position
		elif abs(desired_position[0] - self.drone_gps_coordinates[0]) < 0.2 and abs(desired_position[1] - self.drone_gps_coordinates[1]) < 0.2 and abs(desired_position[2] - self.drone_gps_coordinates[2]) < 0.2: # if near destination
			if not self.is_box_picked: # check box is picked 
					self.state = 1

			elif self.is_box_picked and not self.landing_found: # check landing marker is found
				if not self.is_delivered:
				    self.state = 4
				    self.marker_location = self.drop_location
				else:
					self.landing_found = True
					self.state = 3
					self.get_land_location() # get landing marker location

			if self.state == 3: # check id state is 3
			    	self.get_land_location() # generate marker location
				if abs(self.marker_location[0] - self.drone_gps_coordinates[0]) < 0.5 and abs(self.marker_location[1] - self.drone_gps_coordinates[1]) < 0.5: # verify marker location
				    self.state = 4
	    			    self.marker_location[2] = self.drop_location[2] + 0.3 # change the height of marker location
		
		# check if state is 3
		elif self.state == 3:
		    pass
		
		# if no condition is true change the state to 0
		else:
	    		self.state = 0
	    		
            	if self.state != 1 and self.state != 4 and self.state != 0 and (self.j != 1 or self.j != 0):# check if state is not 1 or 4
    	            for ind, obs in enumerate(self.obsracle_ranges): # check for obstacles
    			if 3 < obs < 8: # check if obstacle found
    			    self.state = 2
    			    self.obstacle_direction = ind + 1
    			    self.avoid_obstacle() # Call function to avoid obstacle
    			    self.obstacle_found = True
    		            self.j = 0
    			    break

    # Based on the direction obstacle found get the coordinate to move
    def after_obtacle_avoided_move_to(self): 
        if self.obstacle_direction == 1:
            return [self.drone_gps_coordinates[0] - 8, self.drone_gps_coordinates[1] + 13, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 2:
            return [self.drone_gps_coordinates[0] + 13, self.drone_gps_coordinates[1] + 8, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 3:
            return [self.drone_gps_coordinates[0] + 8, self.drone_gps_coordinates[1] - 13, self.drone_gps_coordinates[2]]
        elif self.obstacle_direction == 4:
            return [self.drone_gps_coordinates[0] - 13, self.drone_gps_coordinates[1] - 8, self.drone_gps_coordinates[2]]


    # get the precise location of the landing marker
    def get_land_location(self):
        hfov_rad = 1.3962634
        img_width = 400

        x, y, w, h = self.target # get marker dimensions

        focal_length = (img_width/2)/math.tan(hfov_rad/2) # calculate focal length

        # Calculating distance between drone and target (only magnitude)
        self.marker[0] = abs((x + w / 2)*(self.drone_gps_coordinates[2] - self.drop_location[2]) / focal_length)
        self.marker[1] = abs((y + h / 2)*(self.drone_gps_coordinates[2] - self.drop_location[2]) / focal_length)

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
            self.get_next_setpoint(self.above_drop_location)
            self.state = 0

        # If the drone can't grip the box
        else:
            rospy.loginfo('drone decending')
            if abs(self.box_location[2] - self.drone_gps_coordinates[2]) < 0.2:
                self.pid([self.box_location[0], self.box_location[1], self.box_location[2] - 0.1])
            else:
                self.pid(self.box_location) # Move drone towards the box


    # Funtion to avoid obstacle
    def avoid_obstacle(self):
        max_dist = 8 
        min_dist = 0.5
        distance = 13
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
                self.get_next_setpoint(self.above_drop_location) # Check for drop location
            else:
                self.get_next_setpoint(self.above_box_location) # Check for pick location
            self.state = 0 # Change the state to 0
        else:
            if self.is_box_picked:
                self.get_next_setpoint(self.above_drop_location) # Check for drop location
            else:
                self.get_next_setpoint(self.above_box_location) # Check for pick location
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
            v_vector = self.above_drop_location[0] - self.drone_gps_coordinates[0], self.above_drop_location[1] - self.drone_gps_coordinates[1]  # Check for drop location
        else:
            v_vector = self.above_box_location[0] - self.drone_gps_coordinates[0], self.above_box_location[1] - self.drone_gps_coordinates[1] # Check for pick location
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

            if self.state == 0:
                pid_output[i] = self.Kp[0][i] * error[i] + self.Kd[0][i] * self.error_change[i] + self.Ki[0][i] * self.error_sum[i]

            elif self.state == 1 or (self.state == 0 and self.j == 0):
                # Calculating PID output.
                pid_output[i] = self.Kp[1][i] * error[i] + self.Kd[1][i] * self.error_change[i] + self.Ki[1][i] * self.error_sum[i]
            elif self.state == 2:
                pid_output[i] = self.Kp[2][i] * error[i] + self.Kd[2][i] * self.error_change[i] + self.Ki[2][i] * self.error_sum[i]

            elif self.state == 3:
                pid_output[i] = self.Kp[3][i] * error[i] + self.Kd[3][i] * self.error_change[i] + self.Ki[3][i] * self.error_sum[i]
            else:
                # Calculating PID output.
                pid_output[i] = self.Kp[4][i] * error[i] + self.Kd[4][i] * self.error_change[i] + self.Ki[4][i] * self.error_sum[i]

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
        if abs(setpoint_gps_coordinates[0] - self.drone_gps_coordinates[0]) < 0.195 and abs(setpoint_gps_coordinates[1] - self.drone_gps_coordinates[1]) < 0.195 and abs(setpoint_gps_coordinates[2] - self.drone_gps_coordinates[2]) < 0.195:
            self.j += 1 # call modify state
            if self.state == 2 and self.j == 1: # if state is 2 and j is 1
                for ind, obs in enumerate(self.obsracle_ranges): # check if obstacle is still present
		        if 1 < obs < 8:
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
	                    self.get_next_setpoint(self.above_drop_location)
	                else:
	                    self.get_next_setpoint(self.above_box_location)

            if self.state == 2 and self.j == 2: # if the obstacle is completly avoied 
                self.state = 0
                if self.is_box_picked:
                    self.get_next_setpoint(self.above_drop_location)
                else:
                    self.get_next_setpoint(self.above_box_location)

def main():
    if e_drone.state == 0:
    	if not e_drone.is_box_picked: # If box has not been picked
    		e_drone.modify_state(e_drone.above_box_location)
    		e_drone.pid(e_drone.all_next_setpoint[e_drone.j])
    	else:
    		e_drone.modify_state(e_drone.above_drop_location)
    		e_drone.pid(e_drone.all_next_setpoint[e_drone.j])

    elif e_drone.state == 1: # pick up the box
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

    e_drone.get_minimum_distance()
    e_drone.get_next_setpoint(e_drone.above_box_location) # Generate setpoints to move towards box location
    # print(e_drone.all_next_setpoint)

    while not rospy.is_shutdown():
        try:

            main()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass

print