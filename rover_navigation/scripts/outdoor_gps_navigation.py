#!/usr/bin/env python3

__author__ = "Debrup"

import rospy
import actionlib
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geodesy.utm import fromLatLong
from utils import *

class GPSGotoCoords:
        def __init__(self, filePath):
                rospy.init_node('gps_goto_coords')
                rospy.on_shutdown(self.on_shutdown)

                # setup tf listener
                self.tfBuffer = tf2_ros.Buffer()
                self.listener = tf2_ros.TransformListener(self.tfBuffer)

                # set the frame id of the movebase goal
                self.movebase_goal = MoveBaseGoal()
                self.movebase_goal.target_pose.header.frame_id = "utm"
                
                # get the array of GPS waypoints to be traversed
                self.waypoint_arr = getGoals(filePath)
               
                # initialise movebase server and connect to it
                self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
                rospy.loginfo("Waiting for the move_base action server to come up")
                self.move_base.wait_for_server()
                rospy.loginfo("Got move_base action server")

                # send the rover to each waypoint one by one
                for waypoint in self.waypoint_arr:
                        lat, long = waypoint
                        self.go_to_goal(lat, long, self.movebase_goal)
                        rospy.sleep(1)
                
                rospy.loginfo("Traversed all waypoints!")
                rospy.signal_shutdown("Navigation finished")

        def go_to_goal(self, lat, lon, goal):
                rospy.loginfo("Moving to (%f, %f)" % (lat, lon))
                
                # get the utm coordinates
                point = fromLatLong(lat, lon)

                # get the current position
                pos = self.tfBuffer.lookup_transform("utm", 'base_link', rospy.Time(0), rospy.Duration(2.0))

                # build the movebase goal using the target point and current position
                buildGoal(pos, point, goal)
                
                # send the built goal and wait for the result
                self.move_base.send_goal(goal)
                success = self.move_base.wait_for_result()

                if not success:
                        self.move_base.cancel_goal()
                        rospy.logwarn("Failed to reach (%f, %f). Error: %d" % (lat, lon, self.move_base.get_state()))
                else:
                        rospy.loginfo("The base moved to (%f, %f)" % (lat, lon))
                
        def on_shutdown(self):
                rospy.loginfo("Canceling all goals")
                self.move_base.cancel_all_goals()


if __name__ == "__main__":
        try:
                GPSGotoCoords(filePath='/home/kratos/cyborg_ws/src/robofest_rover/rover_navigation/waypoints.csv')
        except rospy.ROSInterruptException:
                rospy.loginfo("Script terminated")