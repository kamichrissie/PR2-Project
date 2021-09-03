# ChrissieRobotics

## Introduction

It is important to understand the different concepts / layers that make possible for a robot to move autonomously.

- **Perception**: knowing the map (occupancy grid), the static and dynamic obstacle.
Being able tyo calculate if the robot would be in a collision state give a certain pose.

- **Localization**: knowing where the robot is.

- **Path planning**: given the current position (A) and a goal (B), calculate a collision-free path to reach B from A.

- **Path following**: convert the output of the path planning into a command to the actuators of the robot in a closed loop.

## Note and links form 2020/10/15

- For didactic purpose, have a look to Path Planning algorithms in [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics). very easy to read!
- An efficient implementation of **A*** here: https://github.com/Eurecat/astar-gridmap-2d/
- For path following, a well known algorithm is **Pure Pursuit** (very useful for car-like robots that can not rotate in place); but probaly an overkill for the time being. Good paper [here](https://www.researchgate.net/publication/220058437_Pure-Pursuit_Reactive_Path_Tracking_for_Nonholonomic_Mobile_Robots_with_a_2D_Laser_Scanner).
- Learn to use RViz and how to add visualization primitives to it (like for instance [your path](https://answers.ros.org/question/278616/how-to-create-a-publisher-about-trajectory-path-then-show-it-in-rviz/)).

# Launch the simulation

     roslaunch gazebo_ros empty_world.launch
     roslaunch pr2_gazebo pr2.launch
     rosrun fake_localization fake_localization _odom_frame_id:=odom_combined

# Task 1

**Goal**: make the robot move to a goal using a [geometry_msg::Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) command.

- No obstacles for now.
- Position of the robot in `/map` frame obtained with [fake_localization](http://wiki.ros.org/fake_localization). Have you launched the node? Have a look at 
[this](https://github.com/dortmans/fake_localization_test).
- Simple straight line path: turn -> move straight -> turn.

Once it works, you can send the goal from RViz using your mouse, as explained [here](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack). It is simply published in `move_base_simple/goal`.

