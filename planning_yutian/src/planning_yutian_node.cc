#include "../include/map_processing.h"
#include "../include/kd_tree.h"
#include "../include/rrt_planner.h"
#include "ros/ros.h"
#include <iostream>

int main(int argc, char** argv) {
	//ros::init(argc, argv, "planning_yutian_node");
	//note that int argc and char** argv should be used as the main funciton input
	//otherwise the node init will mash up
	lmk_rrtplanning::RrtPlanner rrt_test(15, 22, 50);
	//test.ROSMapOutput();
	rrt_test.TestShow(argc, argv);
	std::cout << "programm successfully finished" << std::endl;
	//ROS_INFO is time stamped
	return 0;
}