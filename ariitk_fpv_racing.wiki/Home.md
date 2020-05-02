Welcome to the ariitk_fpv_racing wiki!

# Overview of Project
The goal is to make a racing drone which goeas around a track autonomously detecting and passing through frames

---
## Dependencies

This package has the following dependencies:

1. [OpenCV](https://opencv.org) 
2. [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
3. ROS Melodic with the following packages:

    1. [catkin_simple](https://github.com/catkin/catkin_simple)
    2. roscpp
    3. mavros_msgs
    4. geometry_msgs
    5. image_transport
    6. cv_bridge
    7. opencv2
    8. std_msgs
    9. eigen_conversations
    10. message_generation
    11. message_runtime
    12. tf
    13. tf_conversions
    14. sensor_msgs
    15. nav_msgs
    16. dynamic_reconfigure
    
---
## Installation

Create a catkin workspace in home dir ignore if laready done)
```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
```
Clone the repositiory into the src folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/AerialRobotics-IITK/ariitk_fpv_racing
```
Build using ``` catkin build ``` after making sure all the dependencies are met.

---
### Packages

-> **detector** : Detection of frames and its centre using approxPolyDP.

-> **detecetor_msgs** : Custom messages made here
    
-> **detector_ros** : ROS package for detector
    
-> **planner** : State machine declaration and transition table 
    
-> **planner_ros** : ROS package for planner package
    
-> **pose_estimation** : Pose calculation from the centre coordinates of frame
    
-> **pose_estimation_ros** : ROS package for pose estimation
    
-> **simulator** : All the launch files and world files for the racing track

---
### Topics

1. **detector_ros** :
```
    Subscribed :

        /image_raw

    Published:

        /centre_coord
        /thresh_img
        /contours
        /centre_img
```
2. **planner** : 
```
    Subscribed:

        /mavros/local_position/odom
        /centre_coord
        /estimated_coord
        /mavros/state
        /front_coord

    Published:

        /mavros/setpoint_position/local
        /curr_state

    ServiceClients:

        mavros/cmd/arming
        mavros/set_mode
```
3. **pose_estimation_ros** :
```
    Subscribed:

        /centre_coord
        /mavros/local_position/odom

    Published:
        
        /estimated_coord
        /est_rotation
        /front_coord
```
---
### Paramters

1. **detector_ros** :
```
    -> h_min, s_min, v_min : min values for hsv
    -> h_max, s_max, v_max : max values for hsv
    -> canny_ker, canny_lower, canny_upper : canny params
    -> min_contour_area : setting the area for the detection algorithm
```

2. **planner** :
```
    -> frame1, frame2, frame3, frame4 : rough poses for the frames
    -> thresh_dist : a thresholded distance used in motion planning
```
---

