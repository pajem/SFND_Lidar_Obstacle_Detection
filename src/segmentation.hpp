/**
 * @file segmentation.hpp
 * @author paulo morales
 * @brief ground plane segmentation for point cloud
 */

#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include <vector>

#include <pcl/common/common.h>

namespace sfnd {

/**
 * @brief Ransac for a 3D plane model
 *
 * @tparam PointT
 * @param cloud
 * @param maxIterations
 * @param distanceTol
 * @return indices of inliers
 */
template <typename PointT>
std::vector<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                             int maxIterations, float distanceTol);

namespace internal {

/**
 * @brief Calculate distance of 3D point to a plane.
 *
 * @param x of point
 * @param y of point
 * @param z of point
 * @param A in Ax + By + Cz + D = 0
 * @param B in Ax + By + Cz + D = 0
 * @param C in Ax + By + Cz + D = 0
 * @param D in Ax + By + Cz + D = 0
 * @return distance of point to plane
 */
inline float distanceToPlane(float x, float y, float z, float A, float B,
                             float C, float D) {
  return std::fabs(A * x + B * y + C * z + D) /
         std::sqrt(A * A + B * B + C * C);
}

} // namespace internal

} // namespace sfnd

#include "segmentation.ipp"

#endif // SEGMENTATION_HPP
