This README file is to narrate Assignment 2 of Introduction to Robotics Lab

In this assignment we were expected to implement a Robot Navigation system using A*, Dijkstra and RRT algorithms.

My implementation is done by using ROS2 Humble;

Prerequisites:
- Ubuntu 22.04 (Recommended) or Ubuntu 20.04
- ROS2 HUMBLE (Install: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Slam Toolbox & Turtlebot3(Install: https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)

To run the code in nav_controller, follow the steps (Sometimes Errors may Occur, But let's hope not and if you encounter one of them please do not hesitate to ask as I encountered most of them :/)

1) create a ros2 ws; (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
$ colcon build

2) Move nav_controller to  ~/ros2_ws/src
3) Again
$ source install/setup.bash
$ colcon build


4) Let's run simulation;
Launch Gazebo
If you encounter an error look like this:
% [ERROR] [gzclient-2]: process has died [pid , exit code -6, cmd 'gzclient']. %
then you should run or put this to ~/.bashrc:
$ export ROS_DOMAIN_ID=30 #TURTLEBOT3
$ . /usr/share/gazebo/setup.sh
Then launch gazebo
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

Then launch slam toolbox:
$ ros2 launch slam_toolbox online_async_launch.py 

Then launch rviz2 simply;
$ rviz2
OR
$ ros2 launch nav_controller path_follow.launch.py 
(Second one initialize the rviz better)

Then explore the map using;
$ ros2 run turtlebot3_teleop teleop_keyboard 
(As map is small this way is faster to localize the robot)

Then you can run;
$ ros2 run nav_controller control 

and after setting a target you can see the robot goes there then the map be generated.
(RRT is not working well so, A* or Dijkstra is recommended)

