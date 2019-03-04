#include "../include/kd_tree.h"
#include "../include/rrt_planner.h"

int main(int argc, char** argv) {
  // note that int argc and char** argv should be used as the main funciton input
  // otherwise the node init will mash up
  ros::init(argc, argv, "rrt_planner");
  ros::NodeHandle gp;
  lmk_rrtplanning::RrtPlanner rrt_test(20);
  ros::ServiceServer planner = gp.advertiseService("global_path_sender", &lmk_rrtplanning::RrtPlanner::SendGlobalPath, &rrt_test);
  // note why ROS_INFO here will not print 
  // the line above must be included, otherwise ROS_INFO will not work
  ros::spin();
  return 0;
}
