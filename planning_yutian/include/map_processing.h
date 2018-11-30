#ifndef ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_MAPPROCESSING_H_
#define ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_MAPPROCESSING_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <boost/bind.hpp>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace lmk_mapproc{
class MapData {
 public:
	//here take care that "{}" should not be followed after function{},
	//otherwise it return a "magic number"
	MapData():map_length(-1),map_height(-1),ros_map(0) {};
	~MapData() {};
	void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancygird);
	void MetamapCallback(const nav_msgs::MapMetaData::ConstPtr& map_metadata);
	void MapSubscriber(int argc, char** argv);
	void ROSMapOutput();
	int get_map_length();
	int get_map_height();
	std::vector<std::vector<int>> get_ros_map();
 private:
	int map_length;
	int map_height;
	std::vector<std::vector<int>> ros_map;
};
} //namespace lmk_mapproc

#endif // ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_MAPPROCESSING_H_
