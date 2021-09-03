#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
//#include <boost/iterator/iterator_concepts.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "movement");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//  /* This sleep is ONLY to allow Rviz to come up */
	sleep(5.0);
  
	// Part of the robot to move
	moveit::planning_interface::MoveGroupInterface group("right_arm");

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the 
	// end-effector
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 1;
	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success;

	if (1){
		// Now, we call the planner to compute the plan
		// and visualize it.
		// Note that we are just planning, not asking move_group 
		// to actually move the robot.
		success = static_cast<bool>(group.plan(my_plan));

		ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
		/* Sleep to give Rviz time to visualize the plan. */
		sleep(5.0);
	}

	/*
	// Move to the Pose goal (to use it when running on the real robot or Gazebo )
	// ^^^^^^^^^^^^^^^^^^^^^^^*/
	success = static_cast<bool>(group.move());
	ROS_INFO("Moving to goal %s",success?"":"FAILED"); 
	

	// END_TUTORIAL  
	ros::shutdown();  
	return 0; 
}

