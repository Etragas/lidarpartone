/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	std::shared_ptr<Node> root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Choice is to go left, right or insert
		// Want to keep track of depth
		if (root == NULL){
			root = std::make_shared<Node>(point, id);
			return;
		}
		int depth = 0;
		std::shared_ptr<Node> cur_node = root;
		while (1) {
			float cur_node_val = cur_node->point[depth%2];
			float insert_val = point[depth%2];
			if (cur_node_val < insert_val){
				// Go right
				if (cur_node->right == NULL) {
					cur_node->right = std::make_shared<Node>(point, id);
					return;
				}
				cur_node = cur_node->right;
			}
			else {
				if (cur_node->left == NULL) {
					cur_node->left = std::make_shared<Node>(point, id);
					return;
				}
				cur_node = cur_node->left;

			}
			depth += 1;
		}
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

	}

	bool nodeInBox(std::vector<float> target, float distanceTol, std::shared_ptr<Node> node){
		for(std::size_t i = 0; i < target.size(); ++i) {
			if (target[i] - distanceTol > node->point[i] || target[i] + distanceTol < node->point[i]) return false;
		}
		return true;
	}

	bool nodeInCircle(std::vector<float> target, float distanceTol, std::shared_ptr<Node> node){
		float distance = 0;
		for(std::size_t i = 0; i < target.size(); ++i) {
			distance += std::pow(target[i] - node->point[i], 2);
		}
		return (std::sqrt(distance) <= distanceTol);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// While not in box
		// Do alternating x y comparisons until point is in box
		// // If a point is not in the box, pick the partition direction that moves closer to box
		// If a point is in the box, check if in the circle, then add to ids
		// If a point is in the box, then both of its partitions are candidates for inclusion
		std::stack<std::shared_ptr<Node>> frontier;
		frontier.push(this->root);
		int depth = 0;
		while (!frontier.empty()) {
			std::shared_ptr<Node> current_node = frontier.top();
			frontier.pop();
			if (current_node == NULL) continue;

			if (nodeInBox(target, distanceTol, current_node)) {
				if (nodeInCircle(target, distanceTol, current_node)){
					std::cout << "WE have a match with id " << current_node->id << std::endl;
				}
				ids.push_back(current_node->id);
				frontier.push(current_node->right);
				frontier.push(current_node->left);
				depth++;
				continue;
			}
			if (current_node->point[depth%2] < target[depth%2]){
				frontier.push(current_node->right);
			} else {
				frontier.push(current_node->left);
			}
			depth++;
			// Check if node in box
				// If in box, check in circle
				  // If in circle, add to ids
			// Done checks on node, see which children to add
			// If node on one side of box, add opposite side child
			// Else add both
		}
		return ids;
	}
	

};