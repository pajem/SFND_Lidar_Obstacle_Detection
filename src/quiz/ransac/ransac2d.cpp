/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

namespace {

/**
 * @brief Calculate distance of 2D point to a line.
 *
 * @param x of point
 * @param y of point
 * @param A in Ax + By + C = 0
 * @param B in Ax + By + C = 0
 * @param C in Ax + By + C = 0
 * @return distance of point to line
 */
float distancePoint2dToLine(float x, float y, float A, float B, float C) {
	return std::fabs(A * x + B * y + C) / std::sqrt(A * A + B * B);
}

} // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	int cloudSize = cloud->size();
	for (int i = 0; i < maxIterations; ++i)
	{
		// random sample
		int p1Index = std::rand() % cloudSize;
		int p2Index = std::rand() % cloudSize;
		while (p2Index == p1Index)
		{
			p2Index = (std::rand() / RAND_MAX) * (cloudSize - 1);
		}
		auto& p1 = (*cloud)[p1Index];
		auto& p2 = (*cloud)[p2Index];

		// Ax + By + C = 0
		float A = p1.y - p2.y;
		float B = p2.x - p1.x;
		float C = (p1.x * p2.y) - (p2.x * p1.y);
		// get count of inliers
		std::unordered_set<int> currentInliersResult;
		for (int cloudIndex = 0; cloudIndex < cloudSize; ++cloudIndex)
		{
			auto& point = (*cloud)[cloudIndex];
			if (distancePoint2dToLine(point.x, point.y, A, B, C) <= distanceTol) {
				currentInliersResult.insert(cloudIndex);
			}
		}
		// keep iteration w/ most inliers as the result
		if (currentInliersResult.size() > inliersResult.size())
		{
			inliersResult = currentInliersResult;
		}
	}

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	int maxIterations = 50;
	float distanceTolerance = 0.5;
	std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceTolerance);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
