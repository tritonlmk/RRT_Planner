#include "../include/rrt_planner.h"
#include "../include/kd_tree.h"

#define pi 3.1415926

bool lmk_rrtplanning::RrtPlanner::collision_free(const std::vector<double> node) {
  //geometric to make bounding box approximation works
	double hypotenuse = std::sqrt(car_width*car_width + car_length*car_length);
	double inner_angle = std::atan(static_cast<double>(car_width)/static_cast<double>(car_length));
	std::vector<std::vector<double>> car_geometric_module(10, std::vector<double>(2, 0.0));
	// node 1 on box
	car_geometric_module[0][1] = node[1] + (car_length/2)*std::cos(node[2]);
	car_geometric_module[0][0] = node[0] + (car_length/2)*std::sin(node[2]);
	// node 2 on box
	car_geometric_module[1][1] = node[1] + (hypotenuse/2)*std::cos(node[2] - inner_angle);
	car_geometric_module[1][0] = node[0] + (hypotenuse/2)*std::sin(node[2]- inner_angle);
	// node 3 on box
	car_geometric_module[2][1] = car_geometric_module[1][1] - (car_length/3)*std::cos(node[2]);
	car_geometric_module[2][0] = car_geometric_module[1][0] - (car_length/3)*std::sin(node[2]);
	// node 4 on box
	car_geometric_module[3][1] = car_geometric_module[1][1] - (car_length/3)*2*std::cos(node[2]);
	car_geometric_module[3][0] = car_geometric_module[1][0] - (car_length/3)*2*std::sin(node[2]);
	// node 5 on box
	car_geometric_module[4][1] = node[1] - (hypotenuse/2)*std::cos(node[2] + inner_angle);
	car_geometric_module[4][0] = node[0] - (hypotenuse/2)*std::sin(node[2] + inner_angle);
	// node 6 on box
	car_geometric_module[5][1] = node[1] - (car_length/2)*std::cos(node[2]);
	car_geometric_module[5][0] = node[0] - (car_length/2)*std::sin(node[2]);
	// node 7 on box
	car_geometric_module[6][1] = node[1] - (hypotenuse/2)*std::cos(node[2] - inner_angle);
	car_geometric_module[6][0] = node[0] - (hypotenuse/2)*std::sin(node[2] - inner_angle);
	// node 8 on box
	car_geometric_module[7][1] = car_geometric_module[6][1] + (car_length/3)*std::cos(node[2]);
	car_geometric_module[7][0] = car_geometric_module[6][0] + (car_length/3)*std::sin(node[2]);
	// node 9 on box
	car_geometric_module[8][1] = car_geometric_module[6][1] + (car_length/3)*2*std::cos(node[2]);
	car_geometric_module[8][0] = car_geometric_module[6][0] + (car_length/3)*2*std::sin(node[2]);
	// node 10 on box
	car_geometric_module[9][1] = node[1] + (hypotenuse/2)*std::cos(node[2] + inner_angle);
	car_geometric_module[9][0] = node[0] + (hypotenuse/2)*std::sin(node[2] + inner_angle);
	for (int i = 0; i < 10; ++i) {
		// there is a little bit hard code here
	  // int index = static_cast<int>(car_geometric_module[i][1])*1881 + static_cast<int>(car_geometric_module[i][0]);
		int x_index = static_cast<int>(car_geometric_module[i][1]);
		int y_index = static_cast<int>(car_geometric_module[i][0]);
		if (map_data.map_occupancy[y_index][x_index] == 1) {
		  return false;
		} else {}
	}
	return true;
}
std::vector<double> lmk_rrtplanning::RrtPlanner::random_configuration(int picture_height, int picture_length) {
	// note in C++ 11, the random_device generate non-deterministic random number
	// but it depends on systems, Linux/Unix better supported
	// construct a trivial random generator engint from a time-based seed
	std::vector<double> random_configuration(3, 0.0);
	// construct a trivial random generator engine from a time-based seed
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution_x_axis((0 + car_width/2), (picture_length - car_width/2));
	std::uniform_real_distribution<double> distribution_y_axis((0 + car_width/2), (picture_height - car_width/2));
	std::uniform_real_distribution<double> distribution_steering_angle(-pi/2, pi/2);
	random_configuration[1] = distribution_x_axis(generator);
	random_configuration[0] = distribution_y_axis(generator);
	random_configuration[2] = distribution_steering_angle(generator);
	return random_configuration;
}
bool lmk_rrtplanning::RrtPlanner::reach_destination(std::vector<double> node) {
  int distance = static_cast<int>(std::sqrt(std::pow((node[0] - destination[0]), 2) + std::pow((node[1] - destination[1]), 2)));
	if (distance <= min_step)
		return true;
	else
		return false;
}
void lmk_rrtplanning::RrtPlanner::acquire_map_data(int argc, char** argv) {
	lmk_mapproc::MapData map_meta;
  // ros::init(argc, argv, "get_mapdata");
  map_meta.MapSubscriber(argc, argv);
	map_data.map_length = map_meta.get_map_length();
	map_data.map_height = map_meta.get_map_height();
  map_data.map_occupancy = map_meta.get_ros_map();
}
void lmk_rrtplanning::RrtPlanner::TestShow(int argc, char** argv) {
	bool path_found_flag = false;
	double ratio;
	std::vector<double> random_config(3, 0.0), new_config(3, 0.0);
	KDTreeNode* temp_node_pointer;
	KDTreeNode temp_node;
	setDestiInitial({270.0, 450.0, 0.0}, {940.0, 1400,0, 0.0});
	lmk_rrtplanning::KDTree test_tree(initial_configuration, 0);
	acquire_map_data(argc, argv);
	// draw ini and desti point using opencv
	cv::Mat test_show_image = cv::Mat::zeros(map_data.map_height, map_data.map_length, CV_8UC3);
	test_show_image = cv::imread("probablity_map.jpg");
	cv::Point temp_point, adding_point;
	temp_point.x = static_cast<int>(initial_configuration[1]);
	temp_point.y = static_cast<int>(initial_configuration[0]);
  cv::circle(test_show_image, temp_point, 5, cv::Scalar(0, 0, 255), 2);
	temp_point.x = static_cast<int>(destination[1]);
	temp_point.y = static_cast<int>(destination[0]);
	cv::circle(test_show_image, temp_point, 5, cv::Scalar(0, 0, 255), 2);
	// main logic building rapidly exploring random tree
	while(!path_found_flag) {
	  random_config = random_configuration(map_data.map_height, map_data.map_length);
			// draw collision free point
		KDTreeNode random_tree_node(random_config);
		temp_node_pointer = test_tree.find_nearest_neighbour(random_tree_node);
		if (temp_node_pointer == nullptr) {
			std::cout<< "nullptr returned" << std::endl;
			return;
		}
		adding_point.x = static_cast<int>(temp_node_pointer->node_value[1]);
		adding_point.y = static_cast<int>(temp_node_pointer->node_value[0]);
		ratio = min_step/(std::sqrt(std::pow((random_config[0] - temp_node_pointer->node_value[0]), 2) + std::pow((random_config[1] - temp_node_pointer->node_value[1]), 2)));
		new_config[0] = temp_node_pointer->node_value[0] + ratio*(random_config[0] - temp_node_pointer->node_value[0]);
		new_config[1] = temp_node_pointer->node_value[1] + ratio*(random_config[1] - temp_node_pointer->node_value[1]);
		new_config[2] = random_config[2];
		// draw point
		if (collision_free(new_config)) {
      temp_point.x = static_cast<int>(new_config[1]);
			temp_point.y = static_cast<int>(new_config[0]);
			cv::circle(test_show_image, temp_point, 5, cv::Scalar(0, 255, 0), 1);
			temp_node_pointer = new KDTreeNode(new_config);
			test_tree.adding_elements(*temp_node_pointer);
			// delete temp_node_pointer;
			cv::line(test_show_image, temp_point, adding_point, cv::Scalar(0, 255, 0), 1);
		}
		if (reach_destination(new_config)) {
			path_found_flag = true;
		}
	}
  cv::imwrite("test_show_image.jpg", test_show_image);
}
void lmk_rrtplanning::RrtPlanner::path_generation() {

}
void lmk_rrtplanning::RrtPlanner::setDestiInitial(std::vector<double> initial, std::vector<double> desti) {
  destination = desti;
	initial_configuration = initial;
};
int lmk_rrtplanning::RrtPlanner::global_rrt(std::vector<double> initi, std::vector<double> desti, int argc, char** argv) {
  // essential intermediate variables
	return 1;
}
void lmk_rrtplanning::RrtPlanner::getdata() {
  std::cout << "min step is " << min_step << std::endl;
	std::cout << "car width is " << car_width <<std::endl;
	std::cout << "car length is " << car_length << std::endl;
}