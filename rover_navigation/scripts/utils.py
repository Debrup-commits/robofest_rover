import csv
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_geometry_msgs import PointStamped
import utm
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

def LatLongToUTM(lat, long):
    utm_x, utm_y, _, _ = utm.from_latlon(latitude=lat, longitude=long)

    UTM_point_output = PointStamped()
    UTM_point_output.header.frame_id = "utm"
    UTM_point_output.header.stamp = rospy.Time(0)
    UTM_point_output.point.x = utm_x
    UTM_point_output.point.y = utm_y
    UTM_point_output.point.z = 0

    return UTM_point_output

def UTMtoMapPoint(UTM_input):
    map_point_output = PointStamped()
    map_point_output.header.stamp = rospy.Time.now()
    map_point_output.header.frame_id = "map"
    not_done=True

    tf_buffer = tf.Buffer()
    listener = tf.TransformListener(tf_buffer)
    time_now = rospy.Time()

    while not_done:
        try:
            if tf_buffer.can_transform("map", "utm", time_now, rospy.Duration(1.0)):
                map_point_output = tf_buffer.transform(UTM_input, 'map')
                not_done = False
            else:
                rospy.loginfo("Transformation unavailable")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transformation failed!")
            rospy.sleep(0.5)
    
    return map_point_output

def buildGoal(map_point, map_next, last_point):
    goal = MoveBaseGoal()

    # Specify the frame in which the goal will be published
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x
    goal.target_pose.pose.position.y = map_point.point.y

    # Specify heading goal using current and next points
    if not last_point:
        x_curr = map_point.point.x
        y_curr = map_point.point.y
        x_next = map_next.point.x
        y_next = map_next.point.y
        delta_x = x_next - x_curr
        delta_y = y_next - y_curr
        yaw_curr = math.atan2(delta_y, delta_x)

        # Specify orientation using quaternions
        quat = quaternion_from_euler(0, 0, yaw_curr)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
    else:
        goal.target_pose.pose.orientation.w = 1.0

    return goal

