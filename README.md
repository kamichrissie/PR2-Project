# ChrissieRobotics

# Launch the simulation

     roslaunch gazebo_ros empty_world.launch
     roslaunch pr2_gazebo pr2.launch
     rosrun fake_localization fake_localization _odom_frame_id:=odom_combined
     rviz
     

# Functions

The main run function is in "line_extraction_ros.cpp"
It extracts lines from laserscans of the sorrounding using the methods from "line_extraction_node.cpp". Those lines are stored as ellipses, based on their midpoint, radius in both directions (x/y) and angle.

The "test_base_controller.cpp" makes the robot move to a target point given the coordinates. (One example movement is initialized also in the main run function)

The "modulation.cpp" calculates the robot path based on the current position, the current speed and the given ellipses that should be avoided. It's based on the paper: https://link.springer.com/content/pdf/10.1007/s10514-012-9287-y.pdf

# Task

I need to put those three main functions together, so the robot is moving to a random target point behind the obstacles; Is scanning the lines of sorrounding objects and avoiding them by using the modulation (which is defined for ellipses)


