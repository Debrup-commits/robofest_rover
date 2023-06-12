#!/usr/bin/env python3

__author__ = "Debrup"

import rospy
import getch
from std_msgs.msg import Int8

def Key_press_check():
    rospy.init_node("waypoint_collect_status")
    print("...Waypoint collect status script...")
    print('''Once the collect_gps_waypoints node is active:
             -Press 's' to collect waypoint
             -Press 'q' to quit both the 'waypoint_collect_status' and 'collect_waypoints' nodes''')
    
    pub = rospy.Publisher('/waypoint_collect_status', Int8, queue_size=10)
    rate = rospy.Rate(10)

    while True:
        key = getch.getch()

        rospy.loginfo("Key pressed: {}".format(key))

        if key == 's':
            # 1->collect waypoint
            pub.publish(1)
            rospy.loginfo("Collecting waypoint")
        elif key == 'q':
            # 0->terminate both scripts
            pub.publish(0)
            rospy.loginfo("Terminating script")
            rospy.signal_shutdown("User command")
            break
        else:
            rospy.logwarn("Irrelevant Key Press! Only 's' and 'q' key presses allowed!")

        rate.sleep()

if __name__=='__main__':
    Key_press_check()
        