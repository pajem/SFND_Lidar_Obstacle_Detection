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
	static constexpr int AXIS_COUNT = 2;

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
		searchNode(ids, target, distanceTol, root, 0);
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
			int axisIndex = depth % AXIS_COUNT;
			if (value[axisIndex] < parent->point[axisIndex]) {
				// left
				insertNode(parent->left, value, id, depth + 1);
			} else {
				// right
				insertNode(parent->right, value, id, depth + 1);
			}
		}
	}

	void searchNode(std::vector<int>& ids, const std::vector<float>& targetPoint, float tolerance, Node*& currentNode, int depth) {
		if (currentNode == nullptr) {
			// empty tree
			return;
		} else {
			auto& currentPoint = currentNode->point;
			if (isInsideToleranceBox(targetPoint, currentPoint, tolerance)) {
				if (distance(targetPoint, currentPoint) <= tolerance) {
					ids.push_back(currentNode->id);
				}
			}
			int axisIndex = depth % AXIS_COUNT;
			// traverse left
			if (currentPoint[axisIndex] > (targetPoint[axisIndex] - tolerance)) {
				searchNode(ids, targetPoint, tolerance, currentNode->left, depth + 1);
			}
			// traverse right
			if (currentPoint[axisIndex] < (targetPoint[axisIndex] + tolerance)) {
				searchNode(ids, targetPoint, tolerance, currentNode->right, depth + 1);
			}
		}
	}

	bool isInsideToleranceBox(const std::vector<float>& targetPoint, const std::vector<float>& point, float tolerance) {
		bool inside = true;
		for (int i = 0; i < AXIS_COUNT; ++i) {
			auto minValue = targetPoint[i] - tolerance;
			auto maxValue = targetPoint[i] + tolerance;
			inside &= (point[i] >= minValue) && (point[i] <= maxValue);
		}
		return inside;
	}

	float distance(const std::vector<float>& p1, const std::vector<float>& p2) {
		float distance = 0;
		for (int i = 0; i < AXIS_COUNT; ++i) {
			distance += (p1[i] - p2[i]) * (p1[i] - p2[i]);
		}
		return std::sqrt(distance);
	}
};




