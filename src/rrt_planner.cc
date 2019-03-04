#include "../include/rrt_planner.h"

// class RrtPlanner
// construction function
lmk_rrtplanning::RrtPlanner::RrtPlanner():map_nh_("~rrt_planner"),path_fordrawing_({}),pi_(3.1415926) {
	min_step_ = 10;
	real_path_ = {};
	destination_ = {-1.0, -1.0, -1.0};
	initial_configuration_ = {-1.0, -1.0, -1.0};
	acquire_map_data();
}
lmk_rrtplanning::RrtPlanner::RrtPlanner(int step): map_nh_("~rrt_planner"), path_fordrawing_({}),pi_(3.1415926) {
	real_path_ = {};
	min_step_ = step;
	destination_ = {-1.0, -1.0, -1.0};
	initial_configuration_ = {-1.0, -1.0, -1.0};
	acquire_map_data();
}
// sturct construction function
lmk_rrtplanning::ROSMapData::ROSMapData():map_height(-1),map_length(-1) {
	metadata_flag = false;
	mapdata_flag = false;
	resolution = 0.0;
}
lmk_rrtplanning::ROSMapData::~ROSMapData() {}
// member function
bool lmk_rrtplanning::RrtPlanner::SendGlobalPath(rrt_planner::GlobalPath::Request &req, rrt_planner::GlobalPath::Response &res) {
	ROS_INFO("service successfully got in");
	// when you push a number in, declear it outside clearly
	// int8_t a = 15;
	// or else if you use int8[] in std_msgs, mistakes may happen
	geometry_msgs::Pose2D temp_point;
	initial_configuration_[1] = req.initial_configuration.x;
	initial_configuration_[0] = req.initial_configuration.y;
	destination_[1] = req.destination_configuration.x;
	destination_[0] = req.destination_configuration.y;
	if (req.model_flag) {
		ROS_INFO("initial desti got, start planning");
		TestShow();
	} else {
		ROS_INFO("initial desti not got, return a empty vector");
	}
	int size = real_path_.size();
	if (size == 0) {
		return true;
	} else {
		for (int i = 0; i < size; ++i) {
			temp_point.x = real_path_[i][1];
			temp_point.y = real_path_[i][0];
			temp_point.theta = 0.0;
			res.globalpath.push_back(temp_point);
	  }
	  return true;
	}
}
// this function is also used for starting the main logic
void lmk_rrtplanning::RrtPlanner::acquire_map_data() {
	int size(0);
	while (!map_data_.metadata_flag) {
		ros::ServiceClient map_meta_client = map_nh_.serviceClient<map_proc::MapMetaP>("/map_metadata_planner");
	  map_proc::MapMetaP map_meta;
	  if (map_meta_client.call(map_meta)) {
		  map_data_.map_length = map_meta.response.length;
		  map_data_.map_height = map_meta.response.height;
		  map_data_.resolution = map_meta.response.resolution;
	  } else {ROS_INFO("failed to call map metadata");}
	  if (map_data_.map_height != -1 && map_data_.map_length != -1) {
		  map_data_.metadata_flag = true;
			ROS_INFO("meta map successfully received");
	  } else {
		  map_data_.metadata_flag = false;
	  }
	}
	ROS_INFO("map height is %d" , map_data_.map_height);
  ROS_INFO("map length is %d" , map_data_.map_length);
	std::vector<std::vector<int>> map_copy(map_data_.map_height, std::vector<int>(map_data_.map_length, -1));
	map_data_.map_occupancy = map_copy;
	// call service to receive occupancygird
	ros::ServiceClient map_client = map_nh_.serviceClient<nav_msgs::GetMap>("/static_map");
	nav_msgs::GetMap map_rawdata;
	while(!map_data_.mapdata_flag) {
		if (map_client.call(map_rawdata)) {
			ROS_INFO("start receiving occupancy grid");
			size = (map_rawdata.response.map.data).size();
			ROS_INFO("map size is %d \n", size);
			map_data_.mapdata_flag = true;
			// change "occupied" and "unknown" thresh into "occupied"
			for (int i = 0; i < size; ++i) {
				int x_index = i%map_data_.map_length;
				int y_index = i/map_data_.map_length;
				if (map_rawdata.response.map.data[i] == -1) {
					map_data_.map_occupancy[y_index][x_index] = 1;
				} else if (map_rawdata.response.map.data[i] == 0) {
					map_data_.map_occupancy[y_index][x_index] = 0;
				} else if (map_rawdata.response.map.data[i] == 100) {
					map_data_.map_occupancy[y_index][x_index] = 1;
				} else {}
			}
	  } else {ROS_INFO("failed to receive map data");}
	}
	tiguan_movebase::VehicleMoveBase get_vehicle_parameters;
	car_width_ = (get_vehicle_parameters.get_car_width()/map_data_.resolution);
	car_length_ = (get_vehicle_parameters.get_car_length()/map_data_.resolution);
	ROS_INFO("car length is %f, car width is %f", car_length_, car_width_);
}
// functions used in rrt planning
bool lmk_rrtplanning::RrtPlanner::collision_free(const std::vector<double> node) {
	// hard code here
	double radius = 13.0;
	double y_index1, y_index2, x_index;
	int step_length = 4;
	for (int i = 0; i < step_length; ++i) {
		x_index = node[1] + 2*radius/step_length*i - radius;
		double delta_y = std::sqrt(radius*radius - std::pow((node[1] - x_index), 2));
		y_index1 = node[0] + delta_y;
		y_index2 = node[0] - delta_y;
		int y_index3 = static_cast<int>(y_index1);
		int y_index4 = static_cast<int>(y_index2);
		int x_index1 = static_cast<int>(x_index);
		if (x_index1 >= (map_data_.map_length-1) || (y_index3 > map_data_.map_height-1) || y_index4 > (map_data_.map_height-1)) {
		return false;
	  } else if (map_data_.map_occupancy[y_index3][x_index1] == 1 || map_data_.map_occupancy[y_index4][x_index1] == 1) {
			return false;
		}
	}
	return true;
}
bool lmk_rrtplanning::RrtPlanner::truning_counterclock(const std::vector<double> node, KDTreeNode* origin) {}
bool lmk_rrtplanning::RrtPlanner::nonholomonic_constriants() {
	return true;
}
// generate initial/destinal configuration if set_flag is set to true
// generate random configuration to search path if set_flag is set to false
std::vector<double> lmk_rrtplanning::RrtPlanner::random_configuration(int picture_height, int picture_length, bool set_flag) {
	// note in C++ 11, the random_device generate non-deterministic random number
	// but it depends on systems, Linux/Unix better supported
	// construct a trivial random generator engint from a time-based seed
	std::vector<double> random_configuration(2, 0.0);
	// construct a trivial random generator engine from a time-based seed
	int trick;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution_x_axis((0 + car_width_/2), (picture_length - car_width_/2));
	std::uniform_real_distribution<double> distribution_y_axis((0 + car_width_/2), (picture_height - car_width_/2));
	random_configuration[1] = distribution_x_axis(generator);
	random_configuration[0] = distribution_y_axis(generator);
	// trick: 10% return the destination configuration
	std::uniform_int_distribution<int> targeting_destination(0,13);
	trick = targeting_destination(generator);
	if (set_flag) {
		return random_configuration;
	} else {
		if (trick == 2) {
		  return destination_;
	  } else {
		  return random_configuration;
	  }
	}
}
bool lmk_rrtplanning::RrtPlanner::reach_destination(std::vector<double> node) {
  int distance = static_cast<int>(std::sqrt(std::pow((node[0] - destination_[0]), 2) + std::pow((node[1] - destination_[1]), 2)));
	if (distance <= min_step_)
		return true;
	else
		return false;
}
// set destination & initial configuration
void lmk_rrtplanning::RrtPlanner::setDestiInitial(std::vector<double> initial, std::vector<double> desti) {
  destination_ = desti;
	initial_configuration_ = initial;
};
// overloaded function, generate random initial/destination configuration
void lmk_rrtplanning::RrtPlanner::setDestiInitial() {
	std::vector<double> temp_configuration(2, 0.0);
	bool initi_set = false;
	bool desti_set = false;
	while(!initi_set) {
		temp_configuration = random_configuration(map_data_.map_height, map_data_.map_length, true);
		if (collision_free(temp_configuration)) {
			initial_configuration_ = temp_configuration;
			initi_set = true;
		} else {}
	}
	while (!desti_set) {
		temp_configuration = random_configuration(map_data_.map_height, map_data_.map_length, true);
		if (collision_free(temp_configuration)) {
			destination_ = temp_configuration;
			desti_set = true;
		} else {}
	}
}
// main logic
void lmk_rrtplanning::RrtPlanner::TestShow() {
	bool path_found_flag = false;
	std::vector<double> random_config(2, 0.0), new_config(2, 0.0);
	// Tree nodes and pointer pointing to Tree nodes
	KDTreeNode* temp_node_pointer;
	KDTreeNode* nearest_node_pointer;
	KDTreeNode random_tree_node;
	//setDestiInitial();
	//ROS_INFO("initial x y theta is: %f, %f, %f", initial_configuration_[1], initial_configuration_[0], initial_configuration_[2]);
	//ROS_INFO("desti x y theta is: %f, %f, %f", destination_[1], destination_[0], destination_[2]);
	if (destination_[0] == -1.0 || initial_configuration_[0] == -1.0) {
		ROS_ERROR("no desti & initi config got, need to set randomly");
		return;
	} else {
		ROS_INFO("initial x y theta is: %f, %f, %f", initial_configuration_[1], initial_configuration_[0], initial_configuration_[2]);
		ROS_INFO("desti x y theta is: %f, %f, %f", destination_[1], destination_[0], destination_[2]);
	}
	if (!collision_free(initial_configuration_)) {
		ROS_ERROR("initial point is in collision, program returned");
		return;
	} else if (!collision_free(destination_)) {
		ROS_ERROR("destination is in collision, please pick another destination");
		return;
	} else {}
	lmk_rrtplanning::KDTree rrt_tree(initial_configuration_, 0, initial_configuration_.size());
	// main logic building rapidly exploring random tree
	while(!path_found_flag) {
		//generate random cspace configuration
		random_config = random_configuration(map_data_.map_height, map_data_.map_length, false);
		while (!collision_free(random_config)) {
			random_config = random_configuration(map_data_.map_height, map_data_.map_length, false);
			random_tree_node.node_value = random_config;
			// add new turning counter_clockwise logic here
			nearest_node_pointer = rrt_tree.find_nearest_neighbour(random_tree_node);
		}
		random_tree_node.node_value = random_config;
		nearest_node_pointer = rrt_tree.find_nearest_neighbour(random_tree_node);
		if (nearest_node_pointer == nullptr) {
			ROS_ERROR("nullptr returned");
			return;
		}
		// generating new
		new_config = tiguan_steering(random_config, nearest_node_pointer);
		// if collision free, else do nothing
		if (collision_free(new_config)) {
			// generating kd-tree and rapidly-exploring random tree
			// think: why new operator must be used here?????
			temp_node_pointer = new KDTreeNode(new_config);
			if (temp_node_pointer == nullptr) {
				ROS_ERROR("nullptr generated when creating new node");
				return;
			}
      temp_node_pointer->parent = nearest_node_pointer;
			rrt_tree.adding_elements(*temp_node_pointer);
			temp_node_pointer->parent = nearest_node_pointer;
			// delete temp_node_pointer;
		} else {}
		if (reach_destination(new_config)) {
			path_found_flag = true;
		}
	}
	ROSMapOutputTest();
	// generating path
	path_generator(temp_node_pointer);
	path_smooth();
	// little bit hard code here
	shortcut(4);
	draw_final_path();
	ROS_INFO("program successfully finished");
}
void lmk_rrtplanning::RrtPlanner::draw_final_path() {
	std::string img_temp_path;
	std::vector<int> temp_storage(2, 0);
	cv::Point temp_point, adding_point;
	cv::Mat test_show_image = cv::Mat::zeros(map_data_.map_height, map_data_.map_length, CV_8UC3);
	if(!ros::param::get("rrt_planner/base_image_location", img_temp_path)) {
		ROS_WARN("failed to get ros param: base img location");
		img_temp_path = "/home/mingkun/catkin_ws/src/rrt_planner/assets/probability_map.jpg";
	}
	test_show_image = cv::imread(img_temp_path);
	int path_size = real_path_.size();
	if (path_size <= 1) {
		ROS_WARN("no path generated");
		return;
	} else {
		for (int i = 0; i < path_size; ++i) {
			temp_storage[0] = static_cast<int>(real_path_[i][0]);
			temp_storage[1] = static_cast<int>(real_path_[i][1]);
			path_fordrawing_.push_back(temp_storage);
		}
	}
	for (int i = 1; i < path_size; ++i) {
		temp_point.x = path_fordrawing_[i][1];
		temp_point.y = path_fordrawing_[i][0];
		cv::circle(test_show_image, temp_point, 13, cv::Scalar(0, 0, 255), 1);
		adding_point.x = path_fordrawing_[i-1][1];
		adding_point.y = path_fordrawing_[i-1][0];
		cv::line(test_show_image, temp_point, adding_point, cv::Scalar(0, 0, 255), 1);
	}
	temp_point.x = path_fordrawing_[path_size-1][1];
	temp_point.y = path_fordrawing_[path_size-1][0];
	cv::circle(test_show_image, temp_point, 13, cv::Scalar(0, 255, 0), 2);
	temp_point.x = path_fordrawing_[0][1];
	temp_point.y = path_fordrawing_[0][0];
	cv::circle(test_show_image, temp_point, 13, cv::Scalar(255, 0, 0), 2);
	if (!ros::param::get("rrt_planner/output_image_location", img_temp_path)) {
		ROS_WARN("failed to ros param: output img location");
		img_temp_path = "/home/mingkun/catkin_ws/src/rrt_planner/assets/test_show_image.jpg";
	}
	std::vector<int> map_center = {static_cast<int>(map_data_.map_height/2), static_cast<int>(map_data_.map_length/2)};
	cv::imwrite(img_temp_path, test_show_image);
};
void lmk_rrtplanning::RrtPlanner::path_generator(KDTreeNode* pointer) {
	std::vector<std::vector<double>> temp_storage;
	int size;
	temp_storage.push_back(destination_);
	while(pointer->parent != nullptr) {
		temp_storage.push_back(pointer->node_value);
		pointer = pointer->parent;
	}
	size = temp_storage.size();
	for (int i = (size-1); i > -1; --i) {
		real_path_.push_back(temp_storage[i]);
	}
};
int lmk_rrtplanning::RrtPlanner::global_rrt(std::vector<double> desti) {
  // essential intermediate variables
	return 1;
}
void lmk_rrtplanning::RrtPlanner::shortcut(int times_parameter) {
  int path_size = real_path_.size();
  std::vector<std::vector<double>> config_storage;
  std::vector<double> temp_point(2, 0.0);
  double distance;
  bool shortcut_flag;
  for (int i = 0; i < (times_parameter*path_size); ++i) {
		shortcut_flag = true;
    path_size = real_path_.size();
    if (path_size < 4) {
      ROS_INFO("too short path, no need to shortcut");
      return;
    }
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> point_one(1, path_size-1);
    std::uniform_int_distribution<int> point_two(1, path_size-1);
    int start_point = point_one(generator);
    int end_point = point_two(generator);
		if (start_point > end_point) {
			std::swap(start_point, end_point);
		}
    distance = std::sqrt(std::pow((real_path_[end_point][0] - real_path_[start_point][0]), 2) +
    std::pow(real_path_[end_point][1] - real_path_[start_point][1], 2));
    int shortcut_length = static_cast<int>(distance/min_step_);
    if (shortcut_length == 1) {
      continue;
    }
    for (int inter = 0;inter < shortcut_length; ++inter) {
      temp_point[0] = real_path_[start_point][0] + (real_path_[end_point][0] - real_path_[start_point][0])/shortcut_length*inter;
      temp_point[1] = real_path_[start_point][1] + (real_path_[end_point][1] - real_path_[start_point][1])/shortcut_length*inter;
      if (!collision_free(temp_point)) {
        shortcut_flag = false;
        config_storage.clear();
        break;
	    } else {
        config_storage.push_back(temp_point);
	    }
    }
    if (shortcut_flag) {
			int cut_length_test = start_point - end_point;
	    real_path_.erase((real_path_.begin() + start_point), (real_path_.begin() + end_point));
			int test_new_size = real_path_.size();
	    int new_size = config_storage.size();
	    for (int i = 0; i < new_size; ++i) {
	      real_path_.insert(real_path_.begin() + start_point + i, config_storage[i]);
	    }
	    config_storage.clear();
    } else {}
  }
};
void lmk_rrtplanning::RrtPlanner::path_smooth() {
	// smooth the path generated
};
std::vector<double> lmk_rrtplanning::RrtPlanner::tiguan_steering(std::vector<double> random_config, KDTreeNode* nearest_node_pointer) {
	std::vector<double> new_config(2, 0.0);
	double ratio;
	ratio = min_step_/(std::sqrt(std::pow((random_config[0] - nearest_node_pointer->node_value[0]), 2) + 
	std::pow((random_config[1] - nearest_node_pointer->node_value[1]), 2)));
	new_config[0] = nearest_node_pointer->node_value[0] + ratio*(random_config[0] - nearest_node_pointer->node_value[0]);
	new_config[1] = nearest_node_pointer->node_value[1] + ratio*(random_config[1] - nearest_node_pointer->node_value[1]);
	return new_config;
};
void lmk_rrtplanning::RrtPlanner::ROSMapOutputTest() {
	std::string save_map_path;
	if (!ros::param::get("rrt_planner/base_image_location", save_map_path)) {
		ROS_WARN("failed to ros param: base image location");
		save_map_path = "/home/mingkun/catkin_ws/src/rrt_planner/assets/probability_map.jpg";
	}
  cv::Mat image_yutian_probablity = cv::Mat::zeros(map_data_.map_height, map_data_.map_length, CV_8UC3);
  for (int i = 0; i < map_data_.map_height; ++i) {
    uchar* image_pointer = image_yutian_probablity.ptr(i);
    for (int j = 0; j < map_data_.map_length; ++j) {
      uchar* pixel_pointer = image_pointer;
      if (map_data_.map_occupancy[i][j] == 1) {
      // pixel_pointer[0] = 255;
        image_yutian_probablity.at<cv::Vec3b>(i, j)[0] = 255;
			  image_yutian_probablity.at<cv::Vec3b>(i, j)[1] = 255;
			  image_yutian_probablity.at<cv::Vec3b>(i, j)[2] = 255;
      } else if (map_data_.map_occupancy[i][j] == 0) {
      // pixel_pointer[2] = 0;
        image_yutian_probablity.at<cv::Vec3b>(i, j)[2] = 0;
      } else {
				ROS_ERROR("wrong point got");
			}
      image_pointer += 3;
    }
  }
  cv::imwrite(save_map_path, image_yutian_probablity);
}