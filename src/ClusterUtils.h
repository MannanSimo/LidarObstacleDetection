#ifndef CLUSTERUTILS_H_
#define CLUSTERUTILS_H_

#include "KDTree.h"
#include <unordered_set>
#include <pcl/common/projection_matrix.h>

template <typename PointT, typename TypeName>
void Proximity(Node<PointT, TypeName> &node, std::vector<TypeName> &cluster,
               std::unordered_map<TypeName, bool> &processed,
               KDTree<PointT, TypeName> &tree, float distanceTol);

template <typename PointT, typename TypeName>
std::vector<std::vector<TypeName>> euclideanCluster(
    KDTree<PointT, TypeName> &tree, float distanceTol, int minSize,
    int maxSize);

template <typename PointT>
std::vector<int> ransacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                             int maxIterations, float distanceTol);

#endif
