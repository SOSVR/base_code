# SOSVR2016 - move_controller
A package for navigating robot using [geometry_msgs](http://wiki.ros.org/geometry_msgs). This package works with both keyboard and joystick.
This package is a subscriber from [joy](http://wiki.ros.org/joy) and a publisher on *cmd_vel* using [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages
### Usage
1. [Install ROS](http://wiki.ros.org/ROS/Installation)

2. Clone this repository in your ROS workspace. Then *make* this package with `catkin_make` and *source* it.

3. Open new terminal
```
roscore
```
4. Open another terimal

```
roslaunch move_controller teleop.joy
``` 
Now you can use joystick to navigate your robot, if you dont have a robot or simulator, you can check how it works using rqt

5. Open another terminal

```
rqt
``` 
You can add topic monitor plugin and monitor *cmd_vel* topic values.
