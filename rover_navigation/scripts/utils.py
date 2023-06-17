__author__ = "Debrup"

import csv
import rospy
import tf2_ros as tf
from tf.transformations import quaternion_from_euler
import math

def wipe_csv(filename):
    # Overwrite the file with an empty CSV
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([])

    print("CSV file wiped off, ready for collection of a new set of points")

def addGPSWaypoint(filename, curr_waypoint, last_waypoint):
    lat_last, long_last = last_waypoint
    success = 1

    if(lat_last == 0 and long_last == 0):
        writeWaypointToFile(filename, curr_waypoint)
    else:
        # check if the waypoints are far enough from each other
        if isSeparationEnough(curr_waypoint, last_waypoint):
            writeWaypointToFile(filename, curr_waypoint)
        else:
            success = 0
            rospy.logwarn("Waypoints are too close to each other! Please move the vehicle furthur and try again")

    return success

def isSeparationEnough(curr_waypoint, last_waypoint):
    lat_curr, long_curr = curr_waypoint
    lat_last, long_last = last_waypoint

    # minimum distance b/w 2 waypoints
    min_separation = 1e-5
    if(abs(lat_curr-lat_last)>min_separation or abs(long_curr-long_last)>min_separation): return 1
    else: return 0

def writeWaypointToFile(filename, curr_waypoint):
    # Open the CSV file in append mode
    with open(filename, 'a', newline='') as file:
        # Create a CSV writer object
        writer = csv.writer(file)

        row = [curr_waypoint[0], curr_waypoint[1]]
        # Write the row
        writer.writerow(row)

        rospy.loginfo("Waypoint Added : Latitude: {} Longitude: {}".format(curr_waypoint[0], curr_waypoint[1]))

def updateFromCSV(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        rows = list(reader)

    if len(rows) == 0:
        return 0, 0

    last_row = rows[-1]
    return float(last_row[0]), float(last_row[1])

def isCSVEmpty(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        try:
            # Attempt to read the first row
            first_row = next(reader)
        except StopIteration:
            # If StopIteration is raised, the file is empty
            return True
    return False

def getGoals(filename):
    goals = []

    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) == 2:
                # get latitude longitude
                lat, long = float(row[0]), float(row[1])
                goals.append([lat, long])

    return goals

def buildGoal(pos, point, goal)->None:
    # calculate angle between current position and goal
    angle_to_goal = math.atan2(point.northing - pos.transform.translation.y, point.easting - pos.transform.translation.x)
    odom_quat = quaternion_from_euler(0, 0, angle_to_goal)

    # set the goal message(Time stamp and target odometry to be achieved)
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point.easting
    goal.target_pose.pose.position.y = point.northing
    goal.target_pose.pose.orientation.x = odom_quat[0]
    goal.target_pose.pose.orientation.y = odom_quat[1]
    goal.target_pose.pose.orientation.z = odom_quat[2]
    goal.target_pose.pose.orientation.w = odom_quat[3]
