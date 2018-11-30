#ifndef ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_KD_TREE_H_
#define ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_KD_TREE_H_

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace lmk_rrtplanning {
//tree node
struct KDTreeNode {
	// node value
	std::vector<double> node_value;
	int axis; // splitting plane of the node
	// parents and childs
	KDTreeNode* left_child;
	KDTreeNode* right_child;
	KDTreeNode* parent;
	// constructor when nothing is given
	KDTreeNode():left_child(nullptr), right_child(nullptr), parent(nullptr), axis(0) {};
	// constructor when node value is given
	KDTreeNode(std::vector<double> node_value_input, int splitting_plane);
	KDTreeNode(std::vector<double> node_value_input);
	// destructor
	~KDTreeNode() {};
};
//functions used on treee
class KDTree {
 public:
	KDTree(std::vector<double> initial, int splitting_plane):kd_tree_root(initial, 0) {};
	~KDTree();
	// functions for kd_tree
	KDTreeNode build_kd_tree() {};
	KDTreeNode* find_nearest_neighbour(KDTreeNode new_node);
	void adding_elements(KDTreeNode& new_node);
	void remove_elements(KDTreeNode new_node);
	void balancing_kd_tree();
  void range_search();
 private:
    KDTreeNode kd_tree_root;
	std::vector<KDTreeNode> kdtree_generated;
	void adding_elements(KDTreeNode& new_node, KDTreeNode& root_node);
	void delete_recursively(KDTreeNode current_root);
	KDTreeNode* find_nearest_neighbour(KDTreeNode new_node, KDTreeNode* current_root);
	double two_node_distance(KDTreeNode new_node, KDTreeNode* current_root);
};
}

#endif //ROS_PLANNING_FIRST_TRY_SRC_PLANNING_YUTIAN_INCLUDE_KD_TREE_H_
