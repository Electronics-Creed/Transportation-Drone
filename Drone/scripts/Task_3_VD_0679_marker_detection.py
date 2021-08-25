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


# Class for landing marker
class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        print("start")
        rospy.init_node('pos_2')  # initializing ros node with name 'position_controller'

        # This corresponds to gps coordinates of eDrone
        self.drone_gps_coordinates = [0,0,0]
        self.drone_gps_coordinates_meters = [0, 0] # drone coordinates in metres

        self.i = 0

        # Declaring marker_cmd of message type MarkerData and initializing values
        self.marker_cmd = MarkerData()
        self.marker_cmd.marker_id = 0
        self.marker_cmd.err_x_m = 0.0
        self.marker_cmd.err_y_m = 0.0

        self.drop_location = [[18.9990965928, 72.0000664814, 10.75], [18.9990965925, 71.9999050292, 22.2], [18.9993675932, 72.0000569892, 10.7]]  # Coordinates of the buildings

        self.target = (0,0,0,0) # Holds the values x, y, w and h of the image
        self.marker = [0,0]

        self.logo_cascade = cv2.CascadeClassifier('/home/emyl-omenu/catkin_ws/src/vitarana_drone/data/cascade.xml') # Invoking the cascade classifier

        self.building_pub = rospy.Publisher('/edrone/marker_data', MarkerData, queue_size=1) # Publishing marker_data

        rospy.Subscriber('/edrone/gps', NavSatFix, self.drone_gps_callback)
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 

        self.bridge = CvBridge()

        self.logo = []
    
        time.sleep(2)

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


    def image_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # converting to gray image
            self.logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05) # get all the x,y,w,h values of all the marker 

            for i in range(len(self.logo)): 
                x, y, w, h = self.logo[i] # unpacking self.logo
                area = w * h # calculating area of each bounding box
                
                if area < 1220 and area != 0: # If area of the box detected is less than 1220 
                    # making the center of image (0, 0)
                    x -= 200 
                    y -= 200

                    self.target = x, y, w, h # saving this value logo into target
                    self.get_global() # calculate distance of landing marker from drone

                    break

        except CvBridgeError as e:
            print(e)
            return


    # To get the marker values that need to be published on the marker_data topic
    def get_global(self):
        hfov_rad = 1.3962634
        img_width = 400

        x, y, w, h = self.target
        
        focal_length = (img_width/2)/math.tan(hfov_rad/2)

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
            self.marker[1] = self.marker[1] 
            self.marker[0] = self.marker[0]
        


if __name__ == '__main__':

    e_drone = Edrone() # Edrone object creation
    r = rospy.Rate(2) # Rate of publishing 2 Hz

    while not rospy.is_shutdown():
        try:

            changed = False # to remove sudden change in value of self.i

            if abs(e_drone.drone_gps_coordinates[2] - e_drone.drop_location[e_drone.i][2]) < 1.05 and not changed: # if drone has reached the building
                e_drone.i += 1 # change the building id
                e_drone.i = min(e_drone.i, 2) 
                changed = True
            else:
                changed = False
            
            e_drone.building_pub.publish(e_drone.i + 1, e_drone.marker[0], e_drone.marker[1]) # publish the marker and building id
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
