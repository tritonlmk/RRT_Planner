#ifndef CATKIN_WS_SRC_RRT_PLANNER_INCLUDE_RRTPLANNER_H_
#define CATKIN_WS_SRC_RRT_PLANNER_INCLUDE_RRTPLANNER_H_

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <glog/logging.h>

#include "ros/ros.h"
#include "kd_tree.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose2D.h"
#include "../../tiguan_movebase/include/tiguan_movebase.h"
#include "rrt_planner/GlobalPath.h"
#include "map_proc/MapMetaP.h"
//#include "local_map/start_goal.h"

namespace lmk_rrtplanning {
struct ROSMapData {
  int map_height;
  int map_length;
  float resolution;
  std::vector<std::vector<int>> map_occupancy;
  bool metadata_flag;
  bool mapdata_flag;
	ROSMapData();
	~ROSMapData();
};
class RrtPlanner{
 public:
	// already add some redundency and to eliminate the error caused by static_cast
  RrtPlanner();
  RrtPlanner(int step);
  ~RrtPlanner() {};
  bool SendGlobalPath(rrt_planner::GlobalPath::Request &req, rrt_planner::GlobalPath::Response &res);
  void acquire_map_data();
  std::vector<double> random_configuration(int length, int width, bool set_flag);
  void setDestiInitial(std::vector<double> initial, std::vector<double> desti);
  void setDestiInitial();
  void TestShow();
  int global_rrt(std::vector<double> desti);
 private:
  const double pi_;
  ROSMapData map_data_;
  ros::NodeHandle map_nh_;
  std::vector<double> destination_;
  std::vector<double> initial_configuration_;
  std::vector<std::vector<int>> path_fordrawing_;
  std::vector<std::vector<double>> real_path_;
  int min_step_;
  double car_width_;
  double car_length_;
  void path_smooth();
  void ROSMapOutputTest();
  void draw_final_path();
  void shortcut(int times_parameter);
  void path_generator(KDTreeNode* pointer);
  bool reach_destination(std::vector<double> node);
  bool collision_free(const std::vector<double> node);
  bool truning_counterclock(const std::vector<double> node, KDTreeNode* origin);
  bool nonholomonic_constriants();
  std::vector<double> tiguan_steering(std::vector<double> random_config, KDTreeNode* nearest_node_pointer);
};
}
#endif // CATKIN_WS_SRC_RRT_PLANNER_INCLUDE_RRTPLANNER_H_
