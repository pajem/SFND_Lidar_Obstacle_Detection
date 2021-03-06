/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <cstdlib>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

namespace {

/**
 * @brief Generate random colors
 *
 * @param count
 * @return colors
 */
std::vector<Color> generateRandomColors(int count) {
  std::srand(time(nullptr));
  std::vector<Color> colors;
  for (int i = 0; i < count; ++i) {
    colors.emplace_back(float(std::rand()) / RAND_MAX,
                        float(std::rand()) / RAND_MAX,
                        float(std::rand()) / RAND_MAX);
  }
  return colors;
}

} // namespace

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    double groundSlope = 0;
    auto lidar = std::make_unique<Lidar>(cars, groundSlope);
    auto pointCloud = lidar->scan();

    ProcessPointClouds<pcl::PointXYZ> processor;

    // segmentation
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    auto segmentedCloudPair = processor.SegmentPlane(pointCloud, maxIterations, distanceThreshold);
    auto &obstaclesCloud = segmentedCloudPair.first;
    auto &groundCloud = segmentedCloudPair.second;
    // renderPointCloud(viewer, obstaclesCloud, "obstacles cloud", Color(1, 0, 0));
    renderPointCloud(viewer, groundCloud, "ground cloud", Color(0, 1, 0));

    // clustering
    float clusterTolerance = 1.0;
    int clusterMinSize = 3;
    int clusterMaxSize = 30;
    auto clusters = processor.Clustering(obstaclesCloud, clusterTolerance,
                                         clusterMinSize, clusterMaxSize);
    int clusterId = 0;
    std::vector<Color> colors = generateRandomColors(clusters.size());
    for (auto &cluster : clusters) {
      std::cout << "cluster size ";
      processor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId]);
      Box box = processor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>& processor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {
    // filter
    Eigen::Vector4f roiMinPoint(-10, -5, -2, 1), roiMaxPoint(30, 6, 10, 1);
    auto filteredCloud = processor.FilterCloud(inputCloud, 0.2, roiMinPoint, roiMaxPoint);

    // segmentation
    int maxIterations = 1000;
    float distanceThreshold = 0.3;
    auto segmentedCloudPair = processor.SegmentPlane(filteredCloud, maxIterations, distanceThreshold);
    auto &obstaclesCloud = segmentedCloudPair.first;
    auto &groundCloud = segmentedCloudPair.second;
    // renderPointCloud(viewer, obstaclesCloud, "obstacles cloud", Color(1, 0, 0));
    renderPointCloud(viewer, groundCloud, "ground cloud", Color(0, 1, 0));

    // clustering
    float clusterTolerance = 0.7;
    int clusterMinSize = 3;
    int clusterMaxSize = 100000;
    auto clusters = processor.Clustering(obstaclesCloud, clusterTolerance,
                                         clusterMinSize, clusterMaxSize);
    int clusterId = 0;
    std::vector<Color> colors = generateRandomColors(clusters.size());
    for (auto &cluster : clusters) {
      std::cout << "cluster size ";
      processor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId]);
      Box box = processor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> processor;
    std::vector<boost::filesystem::path> pcdFiles = processor.streamPcd("../src/sensors/data/pcd/data_1");
    auto pcdFileIt = pcdFiles.begin();
    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processor.loadPcd((*pcdFileIt).string());
        cityBlock(viewer, processor, inputCloud);
        ++pcdFileIt;
        if (pcdFileIt == pcdFiles.end()) {
            pcdFileIt = pcdFiles.begin();
        }

        viewer->spinOnce ();
    } 
}