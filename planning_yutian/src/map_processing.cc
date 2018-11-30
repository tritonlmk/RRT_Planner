#include "map_processing.h"

void lmk_mapproc::MapData::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancygrid) {
  int size = (occupancygrid->data).size();
	ROS_INFO("map topic successfully callback");
	std::vector<std::vector<int>> map(map_height, std::vector<int>(map_length, -1));
	ros_map = map;
	for (int i = 0; i < size; ++i) {
		int x_index = i%map_length;
		int y_index = i/map_length;
		// for ros_map, 0 means free thresh, 1 means obstacle
		if (occupancygrid->data[i] == -1) {
			ros_map[y_index][x_index] = 1;
		} else if (occupancygrid->data[i] == 0) {
			ros_map[y_index][x_index] = 0;
		} else if (occupancygrid->data[i] == 100) {
			ros_map[y_index][x_index] = 1;
		}	
	}
}
void lmk_mapproc::MapData::MetamapCallback(const nav_msgs::MapMetaData::ConstPtr& map_metadata) {
  ROS_INFO("meta map topic successfully callback");
	map_length = map_metadata->width;
	map_height = map_metadata->height;
	std::cout << "map height is " << map_height << std::endl;
	std::cout << "map length is " << map_length << std::endl;
}
void lmk_mapproc::MapData::MapSubscriber(int argc, char** argv) {
	ROS_INFO("subscriber successfully used");
  ros::init(argc, argv, "map_reader_node");
	ros::NodeHandle n;
	//note: the fourth parameter of subscribe function is that it should point out which class it belongs to
	//for functions in the same class, it is a "this" pointer
	ros::Subscriber map_meta = n.subscribe("/map_metadata", 10, &MapData::MetamapCallback, this);
	ros::Subscriber map_final = n.subscribe("/map", 100, &MapData::MapCallback, this);
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		if (map_length != -1 && map_height != -1 && ros_map.size() == map_height)
			break;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
void lmk_mapproc::MapData::ROSMapOutput() {
	cv::Mat image_yutian_probablity = cv::Mat::zeros(map_height, map_length, CV_8UC3);
	std::cout << "CV Mat size is " << image_yutian_probablity.size() << std::endl;
	std::cout << "rows and columns" << image_yutian_probablity.rows << image_yutian_probablity.cols << std::endl;
	cv::Mat img_test;
	img_test	= cv::imread("Screen_test.png", 3);
  if(!img_test.data)
		std::cout<< "image not found" << std::endl;
	else
	  std::cout<< "image size is" << img_test.size() << std::endl;
	for (int i = 0; i < map_height; ++i) {
		uchar* image_pointer = image_yutian_probablity.ptr(i);
	  for (int j = 0; j < map_length; ++j) {
			uchar* pixel_pointer = image_pointer;
			if (ros_map[i][j] == -1) {
				// pixel_pointer[0] = 255;
				image_yutian_probablity.at<cv::Vec3b>(i, j)[0] = 0;
			} else if (ros_map[i][j] == 1) {
				// pixel_pointer[1] = 0;
				image_yutian_probablity.at<cv::Vec3b>(i, j)[1] = 255;
			} else if (ros_map[i][j] == 0) {
				// pixel_pointer[2] = 0;
				image_yutian_probablity.at<cv::Vec3b>(i, j)[2] = 255;
			}
			image_pointer += 3;
		}
	}
	cv::imwrite("probablity_map3.jpg", image_yutian_probablity);
	cv::imshow("probablity_map", img_test);
}
int lmk_mapproc::MapData::get_map_length() {return map_length;}
int lmk_mapproc::MapData::get_map_height() {return map_height;}
std::vector<std::vector<int>> lmk_mapproc::MapData::get_ros_map() {return ros_map;}