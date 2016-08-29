behaviour
=====

Behaviour code of S.O.S.VR team, for participating in RoboCup 2016, Leipzig.
This package uses [smach](http://wiki.ros.org/smach), for smach descritption and information you can go through [smach tutorials](http://wiki.ros.org/smach/Tutorials).

##TODO:
This folder will be a ROS package and executing will differ.
State machine used in this smach should be displayed in [##States] part.
Compatible robots should be introduced and be linked.

##Usage

We assume you've installed ROS and Gazebo and configured it correctly. Then:
1. Clone this repository somewhere and navigate there. Then open a new terminal there and run this commands:
```
$ catkin_make
$ source devel/setup.zsh
```
2. For running the smach, cd to package location:
```
$ ./behaviour.py
```
3. The run smach_viewer to check your smach:
```
$ rosrun smach_viewer smach_viewer.py 
```
