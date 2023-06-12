#!/usr/bin/env python3

__author__ = "Debrup"

import rospy
from sensor_msgs.msg import NavSatFix
import csv
from utils import *
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
import actionlib

class GPSNavigate:
    def __init__(self, filepath):
        rospy.init_node('outdoor_gps_navigation')

        self.file_path = filepath
        self.rate = rospy.Rate(10)

        if isCSVEmpty(self.file_path):
            rospy.loginfo("No GPS waypoints.")
            rospy.signal_shutdown('No waypoints')
        
        # at this point, CSV file contains waypoints
        rospy.loginfo("Waypoints received!")

        # variables storing current and next waypoints
        self.lat_curr=0
        self.long_curr=0
        self.lat_next=0
        self.long_next=0

        # get waypoints in array format 
        self.waypoint_arr = getGoals(self.file_path)

        # Create an action client for MoveBase
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        for i in range(len(self.waypoint_arr)):
            
            # set curr waypoint
            self.lat_curr, self.long_curr = self.waypoint_arr[i]
            rospy.loginfo("Target recieved! Latiitude: {} Longitude: {}".format(self.lat_curr, self.long_curr))
            final_point = False

            # set next waypoint if i<len-1
            if i < len(self.waypoint_arr)-1:
                self.lat_next, self.long_next = self.waypoint_arr[i+1]
            # else set next to current
            else:
                self.lat_next, self.long_next = self.waypoint_arr[i]
                final_point = True

            # get UTM point
            UTM_curr = LatLongToUTM(self.lat_curr, self.long_curr)
            UTM_next = LatLongToUTM(self.lat_next, self.long_next)

            # get map point
            map_curr = UTMtoMapPoint(UTM_curr)
            map_next = UTMtoMapPoint(UTM_next)

                        
            # generate goal
            movebase_goal = buildGoal(map_point=map_curr, map_next=map_next, last_point=final_point)
            
            # send goal to movebase
            self.move_base_client.send_goal(movebase_goal)
            self.move_base_client.wait_for_result()

            # Check the status of the goal
            goal_status = self.move_base_client.get_state()

            if goal_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
            else:
                rospy.logwarn("Goal failed!")
                rospy.signal_shutdown("Goal could not be reached, terminating navigation")
            
            self.rate.sleep()

        rospy.loginfo("Successfully traversed all points!")

if __name__ == '__main__':
    try:
        GPSNavigate(filepath='/home/kratos/cyborg_ws/src/robofest_rover/rover_navigation/waypoints.csv')
    except rospy.ROSInterruptException:
        rospy.loginfo("Script terminated")
        