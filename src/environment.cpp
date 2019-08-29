#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"
#include "KDTree.cpp"
#include "ClusterUtils.cpp"
#include "Node.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  Lidar *lidar = new Lidar(cars, 0.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
  renderPointCloud(viewer, cloud, "Point Cloud", Color(0, 1, 0));

  auto *pointcloud_processor = new ProcessPointClouds<pcl::PointXYZ>();
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
      pointcloud_processor->SegmentPlane(cloud, 100, 0.1);
  renderPointCloud(viewer, segmentCloud.first, "Obstacles", Color(1, 1, 1));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      pointcloud_processor->Clustering(segmentCloud.first, 2, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pointcloud_processor->numPoints(cluster);
    renderPointCloud(viewer, cluster,
                     "obstCloudClustered" + std::to_string(clusterId),
                     colors[clusterId]);
    Box box = pointcloud_processor->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
  delete pointcloud_processor;
  delete lidar;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> &pointcloud_processor,
               pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
      pointcloud_processor.FilterCloud(inputCloud, 0.2,
                                       Eigen::Vector4f(-10, -6, -3, 1),
                                       Eigen::Vector4f(20, 6, 2, 1));
  renderPointCloud(viewer, filterCloud, "filteredCloud", Color(0, 1, 0));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
      pointcloud_processor.SegmentPlane(filterCloud, 100, 0.2);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointcloud_processor.Clustering(segmentCloud.first, 0.3, 11, 700);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pointcloud_processor.numPoints(cluster);
    renderPointCloud(viewer, cluster,
                     "obstCloudClustered" + std::to_string(clusterId),
                     colors.at(clusterId % colors.size()));
    Box box = pointcloud_processor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  viewer->initCameraParameters();

  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  auto pointcloud_processor =
      std::make_shared<ProcessPointClouds<pcl::PointXYZI>>();
  std::vector<boost::filesystem::path> stream =
      pointcloud_processor->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    inputCloudI = pointcloud_processor->loadPcd((*streamIterator).string());
    cityBlock(viewer, *pointcloud_processor, inputCloudI);

    streamIterator++;
    if (streamIterator == stream.end()) streamIterator = stream.begin();

    viewer->spinOnce();
  }
}
