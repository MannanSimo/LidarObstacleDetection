#include "processPointClouds.h"

template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  auto startTime = std::chrono::steady_clock::now();

  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered{
      new pcl::PointCloud<PointT>};
  sor.filter(*cloud_filtered);

  typename pcl::PointCloud<PointT>::Ptr CloudRegion{
      new pcl::PointCloud<PointT>};

  pcl::CropBox<PointT> region_of_interest(true);
  region_of_interest.setMin(minPoint);
  region_of_interest.setMax(maxPoint);
  region_of_interest.setInputCloud(cloud_filtered);
  region_of_interest.filter(*CloudRegion);

  std::vector<int> indices;

  pcl::CropBox<PointT> region(true);
  region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  region.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
  region.setInputCloud(CloudRegion);
  region.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices) {
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(CloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*CloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return CloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  typename pcl::PointCloud<PointT>::Ptr obstacles_cloud{
      new typename pcl::PointCloud<PointT>()};
  typename pcl::PointCloud<PointT>::Ptr plane_cloud{
      new typename pcl::PointCloud<PointT>()};

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  extract.setNegative(true);
  extract.filter(*obstacles_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud,
                                                             plane_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  auto startTime = std::chrono::steady_clock::now();

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
  auto ids = ransacPlane<PointT>(cloud, maxIterations, distanceThreshold);
  for (auto i : ids) {
    inliers->indices.push_back(i);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr> segResult =
      SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<
    PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                        float clusterTolerance, int minSize, int maxSize) {
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  KDTree<PointT, int> tree;
  PointT point = cloud->points.at(0);

  for (int index = 0; index < cloud->points.size(); index++) {
    PointT point = cloud->points.at(index);
    tree.insert(point, index);
  }

  std::vector<std::vector<int>> cluster_indices{
      euclideanCluster(tree, clusterTolerance, minSize, maxSize)};

  for (auto &indices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster{
        new pcl::PointCloud<PointT>};
    for (int index : indices) {
      cloud_cluster->points.push_back(cloud->points.at(index));
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  sort(paths.begin(), paths.end());

  return paths;
}
