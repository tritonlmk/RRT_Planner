#ifndef ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_RRTPLANNER_H_
#define ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_RRTPLANNER_H_

#include <vector>
#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"
#include "./kd_tree.h"
#include "./map_processing.h"

namespace lmk_rrtplanning {
struct ROSMapData {
  int map_height;
	int map_length;
	std::vector<std::vector<int>> map_occupancy;
};
class RrtPlanner{
 public:
	// already add some redundency and to eliminate the error caused by static_cast
  RrtPlanner(int step, int cw, int cl): min_step(step),car_width(cw),car_length(cl) {};
	~RrtPlanner() {};
	bool collision_free(const std::vector<double> node);
	std::vector<double> random_configuration(int length, int width);
	bool reach_destination(std::vector<double> node);
	void path_generation();
	int global_rrt(std::vector<double> initi, std::vector<double> desti, int argc, char** argv);
	void acquire_map_data(int argc, char** argv);
	void TestShow(int argc, char** argv);
	void setDestiInitial(std::vector<double> initial, std::vector<double> desti);
	void getdata();
 private:
	ROSMapData map_data;
	std::vector<double> destination;
	std::vector<double> initial_configuration;
	int min_step;
	int car_width;
	int car_length;
  int initi_config[2];
  int desti_config[2];
};
}
#endif // ROS_PLANNING_FIRST_TYR_SRC_PLANNING_YUTIAN_INCLUDE_RRTPLANNER_H_
