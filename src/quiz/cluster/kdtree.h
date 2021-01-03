/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <pcl/common/distances.h>

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;

	Node(PointT arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

template <typename PointT>
struct KdTree
{
	std::shared_ptr<Node<PointT>> root;

	KdTree()
		: root(NULL)
	{
	}

	void insert(PointT point, int id)
	{
		// Choice is to go left, right or insert
		// Want to keep track of depth
		if (root == NULL)
		{
			root = std::make_shared<Node<PointT>>(point, id);
			return;
		}
		int depth = 0;
		std::shared_ptr<Node<PointT>> cur_node = root;
		while (1)
		{
			float cur_node_val = cur_node->point.data[depth % 3];
			float insert_val = point.data[depth % 3];
			if (cur_node_val < insert_val)
			{
				// Go right
				if (cur_node->right == NULL)
				{
					cur_node->right = std::make_shared<Node<PointT>>(point, id);
					return;
				}
				cur_node = cur_node->right;
			}
			else
			{
				if (cur_node->left == NULL)
				{
					cur_node->left = std::make_shared<Node<PointT>>(point, id);
					return;
				}
				cur_node = cur_node->left;
			}
			depth += 1;
		}
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
	}

	bool nodeInBox(PointT target, float distanceTol, std::shared_ptr<Node<PointT>> node)
	{
		for (int i = 0; i < 3; ++i)
		{
			if (!(std::abs(target.data[i] - node->point.data[i]) < distanceTol))
				return false;
		}
		return true;
	}

	bool nodeInCircle(PointT target, float distanceTol, std::shared_ptr<Node<PointT>> node)
	{
		return pcl::euclideanDistance(target, node->point) < distanceTol;
		// float distance = 0;
		// for(std::size_t i = 0; i < 3; ++i) {
		// 	distance += std::pow(target.data[i] - node->point.data[i], 2);
		// }
		// return (std::sqrt(distance) <= distanceTol);
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		// While not in box
		// Do alternating x y comparisons until point is in box
		// // If a point is not in the box, pick the partition direction that moves closer to box
		// If a point is in the box, check if in the circle, then add to ids
		// If a point is in the box, then both of its partitions are candidates for inclusion
		std::stack<std::tuple<std::shared_ptr<Node<PointT>>, int>> frontier;
		int depth;
		std::shared_ptr<Node<PointT>> current_node;
		frontier.push(std::make_tuple(this->root, 0));
		while (!frontier.empty())
		{
			std::tie(current_node, depth) = frontier.top();
			frontier.pop();
			if (current_node == NULL)
				continue;

			if (nodeInBox(target, distanceTol, current_node))
			{
				if (nodeInCircle(target, distanceTol, current_node))
				{
					// std::cout << "WE have a match with id " << current_node->id << std::endl;
					ids.push_back(current_node->id);
				}
			}
			// My code assumed that being in the box was equiv to being on either side
			// But not true because could fail box test in some dimension but not in comparison one
			// Check if splits on left side
			if (std::abs(current_node->point.data[depth % 3] - target.data[depth % 3]) < distanceTol)
			{
				frontier.push(std::make_tuple(current_node->left, depth + 1));
				frontier.push(std::make_tuple(current_node->right, depth + 1));
				continue;
			}
			if (current_node->point.data[depth % 3] < target.data[depth % 3])
			{
				frontier.push(std::make_tuple(current_node->right, depth + 1));
			}
			else
			{
				frontier.push(std::make_tuple(current_node->left, depth + 1));
			}
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