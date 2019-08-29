#include "ClusterUtils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <cmath>

template <typename PointT, typename TypeName>
void Proximity(Node<PointT, TypeName> &node, std::vector<TypeName> &cluster,
               std::unordered_map<TypeName, bool> &processed,
               KDTree<PointT, TypeName> &tree, float distanceTol) {
  processed.at(node.id) = true;

  cluster.push_back(node.id);

  std::vector<TypeName> nearby{tree.search(*(node.data), distanceTol)};
  for (auto p : nearby) {
    if (!processed.at(p)) {
      Proximity(*(tree.get_nodes().at(p)), cluster, processed, tree,
                distanceTol);
    }
  }
}

template <typename PointT, typename TypeName>
std::vector<std::vector<TypeName>> euclideanCluster(
    KDTree<PointT, TypeName> &tree, float distanceTol, int minSize,
    int maxSize) {
  std::vector<std::vector<TypeName>> clusters;

  std::unordered_map<TypeName, bool> processed;

  for (auto p : tree.get_nodes()) {
    processed.insert({p.first, false});
  }

  for (auto p : tree.get_nodes()) {
    if (!processed.at(p.first)) {
      std::vector<TypeName> cluster{};
      Proximity(*(p.second), cluster, processed, tree, distanceTol);
      if ((cluster.size() >= minSize) && (cluster.size() <= maxSize)) {
        clusters.push_back(cluster);
      }
    }
  }
  return clusters;
}

template <typename PointT>
std::vector<int> ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                             int maxIterations, float distanceTol) {
  std::vector<int> inliersResult;
  srand(time(nullptr));

  auto num_points{cloud->points.size()};
  std::mt19937 gen(0);  // Standard mersenne_twister_engine seeded with 0

  // For max iterations
  for (std::size_t iter{0}; iter <= maxIterations; iter++) {
    std::vector<int> inliers_test;
    int index_p1{0}, index_p2{0}, index_p3{0};
    while ((index_p1 == index_p2) || (index_p1 == index_p3) ||
           (index_p2 == index_p3)) {
      std::uniform_int_distribution<int> dis(0, num_points - 1);

      std::vector<int> numbers;

      for (int i = 0; i < 3; i++) {
        numbers.push_back(dis(gen));
      }
      index_p1 = numbers.at(0);
      index_p2 = numbers.at(1);
      index_p3 = numbers.at(2);
    }
    Eigen::Vector3f p_1{cloud->points.at(index_p1).x,
                        cloud->points.at(index_p1).y,
                        cloud->points.at(index_p1).z};
    Eigen::Vector3f p_2{cloud->points.at(index_p2).x,
                        cloud->points.at(index_p2).y,
                        cloud->points.at(index_p2).z};
    Eigen::Vector3f p_3{cloud->points.at(index_p3).x,
                        cloud->points.at(index_p3).y,
                        cloud->points.at(index_p3).z};
    Eigen::Vector3f vec_1{p_2 - p_1};
    Eigen::Vector3f vec_2{p_3 - p_1};
    Eigen::Vector3f normal{(vec_1.cross(vec_2)).normalized()};

    auto d = -normal.dot(p_1);

    for (std::size_t index = 0; index < cloud->points.size(); index++) {
      Eigen::Vector3f v{cloud->points.at(index).x, cloud->points.at(index).y,
                        cloud->points.at(index).z};
      if (std::fabs(normal.dot(v) + d) <= distanceTol) {
        inliers_test.push_back(index);
      }
    }
    if (inliers_test.size() >= inliersResult.size()) {
      inliersResult = std::move(inliers_test);
    }
  }

  return inliersResult;
}
