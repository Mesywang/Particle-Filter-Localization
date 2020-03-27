# The Homework of Probabilistic Robotics 

## Task Description
　　The goal of this homework is to become familiar with robot localization and particle filtering.You will be implementing a global localization for a lost robot (global meaning that you do not know the initial pose of the robot). You may implement this using any programming language (there is no real-time-ness requirement).Feel free to utilize any techniques that we have discussed in class,as well as extension discussed in Probabilistic Robotics or elsewhere.In addition to the readings for lecture, Chapters 5 and 6 of Probabilistic Robotics may be helpful for this assignment.

　　Your lost robot is operating in a building with nothing but odometry and a laser range finder.Fortunately, you have a map of and a deep understanding of particle filtering to help it localize.The data directory that you received with this handout has the following files:

+ instruct.txt –Format description for the map and the data logs.
+ robotdataN.log.gz –Five data logs (odometry and laser data).
+ wean.dat.gz –Map for localization.
+ wean.gif –Image of map (just for your info).
+ robotmovie1.gif –Animation of data log 1 (just for your info).


## 1.Prerequisites
+ Ubuntu 64-bit 16.04. [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## 2.Build on ROS
+ Clone this repository to your catkin workspace and catkin_make.
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Mesywang/Particle-Filter-Localization.git
```
+ Change the name of the folder that you just cloned to "pf_localization" , which matches the name of ROS package.
```
  cd ../
  catkin_make
```
+ Modify the **value** of the parameter **package_path_param** in line 16 of launch/mcl_localization.launch to the path where the pf_localization package is located.  E.g : **value="/home/wsy/catkin_ws/src/"**

+ Run the Simulation : roslaunch pf_localization mcl_localization.launch

+ The result

　　As shown in the following GIF, the pink arrows represent the set of particles, and the motion of black robot model represents the pose of the actual robot estimated by the MCL algorithm in the map.
<div align=center> Monte Carlo Localization </div>
<div align=center>
	<img src="./img/MCL.gif" width = "700" height = "450" >
</div>
