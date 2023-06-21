# robofest_rover
## Software stack for a GPS-based autonomous navigation rover -- by Team NITR

## Dependencies
### Software and frameworks used
- [Ubuntu 20.04](https://www.releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Gazebo](https://gazebosim.org/home)
### Packages used
- [MoveBase](http://wiki.ros.org/move_base)
  - Installation: ``` sudo apt-get install ros-noetic-move-base ```
- [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
  - Installation: ``` sudo apt-get install ros-noetic-robot-localization ```
- [geodesy](https://docs.geodesy.online/)
  - Installation: ``` pip3 install geodesy ```
- [math](https://docs.python.org/3/library/math.html)
  - Installation: ``` pip3 install math ```

## Setting up the repo
### It is assumed that you already have a ROS workspace setup before setting up this repository

### clone the repo in the src of your workspace
```
cd <path to your ROS workspace>/src
git clone https://github.com/Debrup-commits/robofest_rover.git
```

### Build the packages
```
catkin build rover_simulation
catkin build rover_navigation
catkin build rover_localisation
```

### source your bash
```
cd ..
source devel/setup.bash
```
## Running the simulated world
Launch the simulated arena with this command:
```
roslaunch rover_simulation rover_arena.launch
```

## GPS-based localization
Launch the GPS-based localization implemented on the rover with this command:
```
roslaunch rover_navigation rover_localisation.launch 
```

## Starting the navigation pipeline
Launch the complete outdoor navigation pipeline of the rover using this command:
```
roslaunch rover_navigation rover_outdoor_nav.launch
```

## ROS nodes in the package:

### Collect_gps_waypoints
This node saves the rover's live GPS waypoints into a CSV file which is later used to showcase the waypoint following capabilities of the rover

### outdoor_gps_navigation
This node iterates through each waypoint in the CSV file, converts it into the robot's odom frame, and sends it as a movebase goal to the movebase server

### waypoint_collect_status
This node detects key presses and accordingly publishes collect status, based on which the Collect_gps_waypoints saves waypoints into the CSV file
