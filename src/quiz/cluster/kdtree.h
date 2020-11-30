/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertNode(this->root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

private:
	/**
	 * @brief Recursively insert point into kdtree.
	 *
	 * @param parent node
	 * @param value of the point
	 * @param id of the point
	 * @param depth of the tree
	 */
	void insertNode(Node*& parent, const std::vector<float> value, int id, int depth) {
		if (parent == nullptr) {
			// create root node
			parent = new Node(value, id);
		} else {
			int axisIndex = depth % 2; // 2D point
			if (value[axisIndex] < parent->point[axisIndex]) {
				// left
				insertNode(parent->left, value, id, ++depth);
			} else {
				// right
				insertNode(parent->right, value, id, ++depth);
			}
		}
	}
};




