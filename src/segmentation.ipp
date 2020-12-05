/**
 * @file segmentation.ipp
 * @author paulo morales
 * @brief ground plane segmentation for point cloud
 */

#ifndef SEGMENTATION_IPP
#define SEGMENTATION_IPP

#include <cmath>
#include <cstdlib>
#include <ctime>

#include "render/render.h"

namespace sfnd {

template <typename PointT>
std::vector<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                             int maxIterations, float distanceTol) {
  std::vector<int> inliersResult;
  std::srand(std::time(nullptr));
  const int cloudSize = cloud->size();
  for (int i = 0; i < maxIterations; ++i) {
    // random sample
    int p1Index = std::rand() % cloudSize;
    int p2Index = std::rand() % cloudSize;
    int p3Index = std::rand() % cloudSize;
    // ensure sample points are unique
    while (p2Index == p1Index || p2Index == p3Index) {
      p2Index = (std::rand() / RAND_MAX) * (cloudSize - 1);
    }
    while (p3Index == p1Index || p3Index == p2Index) {
      p3Index = (std::rand() / RAND_MAX) * (cloudSize - 1);
    }
    auto &p1 = (*cloud)[p1Index];
    auto &p2 = (*cloud)[p2Index];
    auto &p3 = (*cloud)[p3Index];

    // plane model
    // Ax + By + Cz + D = 0
    Vect3 v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    Vect3 v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
    Vect3 cross = v1 * v2;
    float A = cross.x;
    float B = cross.y;
    float C = cross.z;
    float D = -((A * p1.x) + (B * p1.y) + (C * p1.z));

    // get count of inliers for current plane model
    std::vector<int> currentInliersResult;
    for (int cloudIndex = 0; cloudIndex < cloudSize; ++cloudIndex) {
      auto &point = (*cloud)[cloudIndex];
      if (internal::distanceToPlane(point.x, point.y, point.z, A, B, C, D) <=
          distanceTol) {
        currentInliersResult.push_back(cloudIndex);
      }
    }

    // keep iteration w/ most inliers as the result
    if (currentInliersResult.size() > inliersResult.size()) {
      inliersResult = currentInliersResult;
    }
  }

  return inliersResult;
}

} // namespace sfnd

#endif // SEGMENTATION_IPP
