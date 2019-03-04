#include "../include/kd_tree.h"

// functions realization in KDTreeNode struct
lmk_rrtplanning::KDTreeNode::KDTreeNode(std::vector<double> node_value_input, int splitting_plane):
	left_child(nullptr), right_child(nullptr), node_value({0.0, 0.0, 0.0}) {
  node_value[0] = node_value_input[0];
	node_value[1] = node_value_input[1];
	node_value[2] = node_value_input[2];
	axis = splitting_plane;
	parent = nullptr;
}
lmk_rrtplanning::KDTreeNode::KDTreeNode(std::vector<double> node_value_input):
	left_child(nullptr), right_child(nullptr), node_value({0.0, 0.0, 0.0}) {
	node_value[0] = node_value_input[0];
	node_value[1] = node_value_input[1];
	node_value[2] = node_value_input[2];
	parent = nullptr;
}
// functions realization in KDTree class
// construnction function
lmk_rrtplanning::KDTree::KDTree(std::vector<double> initial, int splitting_plane, int dimension):length_that_could_not_be_surpassed_(5000.0) {
	cspace_dimension = dimension;
	KDTreeNode temp(initial, 0);
	kd_tree_root_ = temp;
}
// destruction function
lmk_rrtplanning::KDTree::~KDTree() {
		delete_recursively(kd_tree_root_.left_child);
		delete_recursively(kd_tree_root_.right_child);
}
// member funciton
lmk_rrtplanning::KDTreeNode* lmk_rrtplanning::KDTree::build_kd_tree() {};
void lmk_rrtplanning::KDTree::delete_recursively(KDTreeNode* current_root) {
	if (current_root == nullptr) {
		return;
	} else {
		delete_recursively(current_root->left_child);
		delete_recursively(current_root->right_child);
		// note, what is the difference between delete and delete[]
		delete current_root;
	}
};
void lmk_rrtplanning::KDTree::adding_elements(KDTreeNode& new_node) {
  adding_elements(new_node, kd_tree_root_);
};
lmk_rrtplanning::KDTreeNode* lmk_rrtplanning::KDTree::find_nearest_neighbour(KDTreeNode new_node) {
  return find_nearest_neighbour(new_node, &kd_tree_root_);
}
void lmk_rrtplanning::KDTree::adding_elements(KDTreeNode& new_node, KDTreeNode& root_node) {
	int splitting_plane = root_node.axis;
	if (new_node.node_value[splitting_plane] < root_node.node_value[splitting_plane]) {
	  if (root_node.left_child == nullptr) {
		  root_node.left_child = &new_node;
			new_node.axis = (splitting_plane + 1)%cspace_dimension;
		} else {
		  adding_elements(new_node, *root_node.left_child);
		}
	} else {
	  if (root_node.right_child == nullptr) {
		  root_node.right_child = &new_node;
			new_node.axis = (splitting_plane + 1)%cspace_dimension;
		} else {
		  adding_elements(new_node, *root_node.right_child);
		}
	}
};
double lmk_rrtplanning::KDTree::two_node_distance(KDTreeNode new_node, KDTreeNode* current_root) {
	double distance;
	if (current_root == nullptr) {
		return length_that_could_not_be_surpassed_;
	} else {
		distance = std::sqrt(std::pow((new_node.node_value[0] - current_root->node_value[0]), 2) + std::pow((new_node.node_value[1] - current_root->node_value[1]), 2));
	}
	return distance;
};
lmk_rrtplanning::KDTreeNode* lmk_rrtplanning::KDTree::find_nearest_neighbour(KDTreeNode new_node, KDTreeNode* current_root) {
  if (current_root == nullptr){
		return nullptr;
	} else if (current_root->left_child == nullptr && current_root->right_child == nullptr) {
		return current_root;
	} else {}
	int splitting_plane = current_root->axis;
	// once reaches a node, checks that node point
	double current_distance = two_node_distance(new_node, current_root);
	double current_best = current_distance;
	// temp parameters
	double distance_to_otherside = std::fabs(new_node.node_value[splitting_plane] - current_root->node_value[splitting_plane]);
	KDTreeNode* nearest_node = current_root;
	KDTreeNode* this_side;
	KDTreeNode* other_side;
	KDTreeNode* current_nearest_node;
	// main logic
	// decide whether to search the right child or the left first
	if (new_node.node_value[splitting_plane] < current_root->node_value[splitting_plane]) {
		this_side = current_root->left_child;
		other_side = current_root->right_child;
	} else {
		this_side = current_root->right_child;
		other_side = current_root->left_child;
	}
  // check if there could be any points on the other side of the splitting plane that are closer to the search point
  // if the hypersphere cross the plane, search the branch of the tree
  if (distance_to_otherside < current_best) {
		current_nearest_node = find_nearest_neighbour(new_node, other_side);
    current_distance = two_node_distance(new_node, current_nearest_node);
	  // if node on the other side is nearee, update the "current best node"
		if (current_distance < current_best) {
		  nearest_node = current_nearest_node;
			current_best = current_distance;
		}
	}
	// continue walking up the tree
	current_nearest_node = find_nearest_neighbour(new_node, this_side);
	current_distance = two_node_distance(new_node, current_nearest_node);
	if (current_distance < current_best) {
		current_best = current_distance;
	  nearest_node = current_nearest_node;
	}
	return nearest_node;
}
