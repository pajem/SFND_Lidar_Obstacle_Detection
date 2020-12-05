/**
 * @file clustering.hpp
 * @author paulo morales
 * @brief point cloud clustering
 */

#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

#include <set>
#include <vector>

#include <pcl/common/common.h>

#include "kdtree.hpp"

namespace sfnd {

template <typename PointT>
void populateClusterFromTree(const typename pcl::PointCloud<PointT> &points,
                             int currentPointIndex, KdTree<PointT> &tree,
                             float distanceTol, std::set<int> &processedIndices,
                             std::vector<int> &cluster) {
  if (processedIndices.count(currentPointIndex) != 0) {
    return;
  }
  processedIndices.insert(currentPointIndex);
  const auto &point = points[currentPointIndex];
  cluster.push_back(currentPointIndex);
  auto closePointIndices = tree.search(point, distanceTol);
  for (auto &closePointIndex : closePointIndices) {
    populateClusterFromTree(points, closePointIndex, tree, distanceTol,
                            processedIndices, cluster);
  }
}

template <typename PointT>
std::vector<std::vector<int>>
euclideanCluster(const typename pcl::PointCloud<PointT> &points,
                 KdTree<PointT> &tree, float distanceTol) {
  std::vector<std::vector<int>> clusters;
  std::set<int> processedIndices;
  for (int i = 0; i < points.size(); ++i) {
    if (processedIndices.count(i) != 0) {
      continue;
    }
    std::vector<int> cluster;
    populateClusterFromTree(points, i, tree, distanceTol, processedIndices,
                            cluster);
    clusters.push_back(cluster);
  }
  return clusters;
}

} // namespace sfnd

#endif // CLUSTERING_HPP
