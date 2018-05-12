# Gas-source-localization-using-a-mobile-Robot

Increasing threats of the leaks of different gases highlight the need for an efficient detection of hazardous emission sources. New odor source detection systems are developed to detect emissions released by different sources, such as fire and toxic gases, in order to prevent different life threating accidents. The aim of the project is to construct an intelligent system on a mobile robot. This system enables the robot to locate harmful gas sources located in fixed place in real indoor environments. We addressed the gas source localization problem with a mobile robot that is equipped with a laser scanner and a gas sensor. The software was deployed on a Raspberry PI microcomputer hosting Robot Operating System (ROS). A Random Sampling approach was implemented on a TurtleBot 2 robot that searches for the location of the highest gas value. The source was found after exploring the room for an average time of 30 minutes with an average final distance of 48 cm between the source and the robot.


**This is a Youtube vedio showing the project running on the  robot.**
https://www.youtube.com/watch?v=x4V3KSUoVfY



**Requirements**

The project has been tested on ROS Kinetic. The following requirements are needed :

1- You should have installed a ROS distribution.

2- Created a workspace.

3- Installed the "gmapping" ROS package: on Ubuntu, and if you are running ROS Kinectic, you can do that by typing the following command in the terminal:

sudo apt-get install ros-kinetic-gmapping

4- Install ROS navigation stack. You can do that with the following command (assuming Ubuntu, ROS Kinetic):
 sudo apt-get install ros-kinetic-navigation

5- You should have Python 2.7.


**Running the codes**

1- first you should run the launch file (base.launch )by using the folowing comand:

roslaunch base.launch

2- write this command which lets you see the map

rosrun rviz rviz

3- then you should build the map by moving the robot using a python script

python go2point.py

4- run the python script

python janaseq.py
