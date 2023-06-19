#!/usr/bin/env python3

__author__ = "Debrup"

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
import csv
import os
import getch
from utils import *


class CollectWaypoints:
    def __init__(self, filename):
        rospy.init_node('collect_gps_waypoints')

        # Variables to store current and previous waypoints
        self.longitude_curr = 0
        self.latitude_curr = 0
        self.longitude_last = 0
        self.latitude_last = 0
        
        # waypoint collect status
        self.waypoint_collect_status = 2

        # Check if the csv file is available. If not, generate one
        if os.path.isfile(filename):
            print("File already exists")
            print("Press '1' if you want to add new waypoints to the existing file")
            print("Press '2' if you want to generate a new list of points")
            print("Press '3' if you want to exit the program")

            choice = int(input("Enter Choice: "))
            
            if choice == 1:
                # update the last waypoint variables
                self.latitude_last, self.longitude_last = updateFromCSV(filename)
            if choice == 2:
                wipe_csv(filename)
            elif choice == 3:
                self.shutdown()
        else:
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([])

            print("New waypoint file generated. Path -> {}".format(filename))

        rospy.Subscriber('/navsat/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/waypoint_collect_status', Int8, self.waypoint_collect_callback)

        rospy.loginfo("Run the waypoint_collect_status ROS node inorder to collect waypoints")

        # Wait for the first GPS fix and the first waypoint selection
        while self.longitude_curr == 0 and self.latitude_curr == 0 and self.waypoint_collect_status == 0:
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            
            if self.waypoint_collect_status == 1:
                success = addGPSWaypoint(filename, [self.latitude_curr, self.longitude_curr], [self.latitude_last, self.longitude_last])
                
                # If waypoint has been added, update current waypoint
                if success:
                    self.latitude_last = self.latitude_curr
                    self.longitude_last = self.longitude_curr

                self.waypoint_collect_status = 2
                rospy.sleep(0.5)

            if self.waypoint_collect_status == 0:
                self.shutdown()
                break
       
    def gps_callback(self, data):
        self.latitude_curr = data.latitude
        self.longitude_curr = data.longitude

    def waypoint_collect_callback(self, data):
        self.waypoint_collect_status = data.data

    def shutdown(self):
        rospy.loginfo("GPS waypoint collection script terminated!")
        rospy.signal_shutdown("User command")

if __name__ == '__main__':
    try:
        CollectWaypoints(filename='/home/kratos/cyborg_ws/src/robofest_rover/rover_navigation/waypoints.csv')
    except rospy.ROSInterruptException:
        rospy.loginfo("Script terminated")
