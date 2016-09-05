#SOSvr Robotic Team Base Code
This is our base code that can be used to build different robots for performing different tasks and simulate them in 3D with Gazebo. We have used ROS for developing our robot and used P3AT as the robot's model.
For setting up this robot refer to the **usage** section.

#Usage
We assume you have an **Ubuntu 14.04** machine set up. Follow these simple steps to bring up the robot: 
###Installing Ros Indigo
First you have to install **ROS indigo**:

####Set up your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
####Set up your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
```
####Installation
```
sudo apt-get update
sudo apt-get install ros-indigo-desktop
```
###Installing Gazebo
For using 3D simulated environment we use Gazebo5. Install it with these commands:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-gazebo5-ros-pkgs
``` 
### Install Some ROS Packages
This packages are needed for the code to be maked and compiled:
#####Controller Manager
```
sudo apt-get install ros-indigo-controller-manager 
```
#####Joy
```
sudo apt-get install ros-indigo-joy
```
#####Gmapping
```
sudo apt-get install ros-indigo-gmapping
```
#####Move Base
```
sudo apt-get install ros-indigo-move-base
```
# Running and compiling
Here we should first create a catkin workspace and compile and run our code. Follow these steps to finally see the result:
###Creating a workspace
Use these commands to create a catkin workspace: (here we name it *catkin_ws*)  
```
source /opt/ros/indigo/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
Then copy the contents of **src** folder from here to the **src folder of your workspace**.
###Compiling the code
Assuming you're in the home directory and your catkin workspace folder is named **catkin_ws** use these commands:
```
cd catkin_ws
catkin_make
``` 
###Running the code:
Use these commands to bring up the 3D environment with the robot spawned in it:
```
roslaunch bring_up server.launch
roslaunch bring_up client.launch
roslaunch bring_up gmapping.launch
roslaunch bring_up move_base.launch
roslaunch bring_up teleop_joy.launch
```
Then Gazebo should be up and running with robot in it :)