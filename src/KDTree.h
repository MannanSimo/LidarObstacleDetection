#ifndef KDTREE_H_
#define KDTREE_H_

#include <unordered_map>
#include "Node.h"

template <typename PointT, typename TypeName>
class KDTree {
  std::shared_ptr<Node<PointT, TypeName>> root;
  std::unordered_map<TypeName, std::shared_ptr<Node<PointT, TypeName>>> nodes;

 public:
  KDTree();

  ~KDTree();

  void insert(PointT &data, TypeName name);

  std::unordered_map<TypeName, std::shared_ptr<Node<PointT, TypeName>>>
  get_nodes();

  std::vector<TypeName> search(PointT &data, float distance_tolerance);
};

#endif
