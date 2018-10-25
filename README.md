# IGVC_2019
IGVC 2019 Team

Welcome to the UT RAS IGVC 2019 Github!

#### ROS Tutorials

> If you have yet to set up a ROS workspace follow these steps: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

> ROS Basic Tutorial: http://wiki.ros.org/ROS/Tutorials
> (I would recommend following along these tutorials until around simple publishers and subscribers if you are new to ROS and linux > in general)

---

#### Dependency List:
> General: roscpp, rospy, std_msgs, tf

> LIDAR: sick_tim, rviz

> Motor: joy (untested)

> Vision: opencv

---
#### Work in Progress:
LIDAR:
* launch rviz, tf simple transform, and sick_tim node all in one launch file
* parse and retrieve parameter_descriptions and parameter_updates
* filter scan data for static objects
* implement singularity transform for self-localization using known landmarks
* apply kalman filtering to lidar localization

MOTOR:
* launch joy_node, serial_node all in one launch file
* read encoder ticks for dead-reckoning based localization
* implement tank and arcade drive

VISION:
* launch camerafeed and opencv all in one launch file
* test the following openCV methods:
  * houghlines
  * dynamic window approach
  * feature-based tracking: e.g. PTAM, ORB-SLAM, etc.
