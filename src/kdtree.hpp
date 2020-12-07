/**
 * @file kdtree.hpp
 * @author paulo morales
 * @brief kdtree implementation
 */

#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <cmath>
#include <vector>

namespace sfnd {

/**
 * @brief KD-tree node
 *
 * @tparam PointT
 */
template <typename PointT> class Node {
public:
  /**
   * @brief Constructor
   *
   * @param setPoint
   * @param setId
   */
  Node(const PointT &setPoint, int setId)
      : point(setPoint), id(setId), left(nullptr), right(nullptr) {}

  PointT point; // point data
  int id;       // node ID
  Node *left;   // left child node
  Node *right;  // right child node
};

template <typename PointT> class KdTree {
public:
  static constexpr int AXIS_COUNT = 3; // XYZ axes

  /**
   * @brief Construct an empty KD-tree.
   *
   */
  KdTree() : root(nullptr) {}

  /**
   * @brief Insert a point in the tree.
   *
   * @param point
   * @param id
   */
  void insert(const PointT &point, int id) {
    insertNode(this->root, point, id, 0);
  }

  /**
   * @brief Return a list of point ids in the tree that are within distance of
   * target
   *
   * @param target point
   * @param distanceTol from target point
   * @return list of point ids close to the target
   */
  std::vector<int> search(const PointT &target, float distanceTol) {
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
  void insertNode(Node<PointT> *&parent, const PointT &value, int id,
                  int depth) {
    if (parent == nullptr) {
      // create root node
      parent = new Node<PointT>(value, id);
    } else {
      int axisIndex = depth % AXIS_COUNT;
      if (value.data[axisIndex] < parent->point.data[axisIndex]) {
        // left
        insertNode(parent->left, value, id, depth + 1);
      } else {
        // right
        insertNode(parent->right, value, id, depth + 1);
      }
    }
  }

  void searchNode(std::vector<int> &ids, const PointT &targetPoint,
                  float tolerance, Node<PointT> *&currentNode, int depth) {
    if (currentNode == nullptr) {
      // empty tree
      return;
    } else {
      auto &currentPoint = currentNode->point;
      if (isInsideToleranceBox(targetPoint, currentPoint, tolerance)) {
        if (distance(targetPoint, currentPoint) <= tolerance) {
          ids.push_back(currentNode->id);
        }
      }
      int axisIndex = depth % AXIS_COUNT;
      // traverse left
      if (currentPoint.data[axisIndex] >
          (targetPoint.data[axisIndex] - tolerance)) {
        searchNode(ids, targetPoint, tolerance, currentNode->left, depth + 1);
      }
      // traverse right
      if (currentPoint.data[axisIndex] <
          (targetPoint.data[axisIndex] + tolerance)) {
        searchNode(ids, targetPoint, tolerance, currentNode->right, depth + 1);
      }
    }
  }

  bool isInsideToleranceBox(const PointT &targetPoint, const PointT &point,
                            float tolerance) {
    bool inside = true;
    for (int i = 0; i < AXIS_COUNT; ++i) {
      auto minValue = targetPoint.data[i] - tolerance;
      auto maxValue = targetPoint.data[i] + tolerance;
      inside &= (point.data[i] >= minValue) && (point.data[i] <= maxValue);
    }
    return inside;
  }

  float distance(const PointT &p1, const PointT &p2) {
    float distance = 0;
    for (int i = 0; i < AXIS_COUNT; ++i) {
      distance += (p1.data[i] - p2.data[i]) * (p1.data[i] - p2.data[i]);
    }
    return std::sqrt(distance);
  }

  Node<PointT> *root; // root node
};

} // namespace sfnd

#endif // KDTREE_HPP
