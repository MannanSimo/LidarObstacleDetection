#include "KDTree.h"
#include <iostream>

template <typename PointT, typename TypeName>
void my_insert(std::shared_ptr<Node<PointT, TypeName>> *current_node,
               PointT data, TypeName name, unsigned int depth,
               std::unordered_map<
                   TypeName, std::shared_ptr<Node<PointT, TypeName>>> &nodes) {
  if (!(*current_node)) {
    (*current_node) = std::make_shared<Node<PointT, TypeName>>(data, name);
    nodes.insert({name, (*current_node)});
  } else if ((*((*current_node)->data)).data[depth % 3] >
             data.data[depth % 3]) {
    current_node = &((*current_node)->left);
    my_insert(current_node, data, name, depth + 1, nodes);
  } else {
    current_node = &((*current_node)->right);
    my_insert(current_node, data, name, depth + 1, nodes);
  }
}

template <typename PointT, typename TypeName>
void my_search(PointT &data, float distanceTol,
               std::shared_ptr<Node<PointT, TypeName>> &current_node,
               unsigned int depth, std::vector<TypeName> &ids) {
  if (current_node) {
    if ((data.x - distanceTol <= (*(current_node->data)).x) &&
        (data.x + distanceTol >= (*(current_node->data)).x) &&
        (data.y - distanceTol <= (*(current_node->data)).y) &&
        (data.y + distanceTol >= (*(current_node->data)).y) &&
        (data.z - distanceTol <= (*(current_node->data)).z) &&
        (data.z + distanceTol >= (*(current_node->data)).z)) {
      ids.push_back(current_node->id);
    }
    if (data.data[depth % 3] - distanceTol <=
        (*(current_node->data)).data[depth % 3]) {
      my_search(data, distanceTol, current_node->left, depth + 1, ids);
    }
    if (data.data[depth % 3] + distanceTol >=
        (*(current_node->data)).data[depth % 3]) {
      my_search(data, distanceTol, current_node->right, depth + 1, ids);
    }
  }
}

template <typename PointT, typename TypeName>
KDTree<PointT, TypeName>::KDTree() {
  std::cout << "KDTree created" << std::endl;
}

template <typename PointT, typename TypeName>
KDTree<PointT, TypeName>::~KDTree() {
  std::cout << "KDTree destroyed" << std::endl;
}

template <typename PointT, typename TypeName>
void KDTree<PointT, TypeName>::insert(PointT &data, TypeName name) {
  my_insert(&root, data, name, 0, nodes);
}

template <typename PointT, typename TypeName>
std::unordered_map<TypeName, std::shared_ptr<Node<PointT, TypeName>>>
KDTree<PointT, TypeName>::get_nodes() {
  return nodes;
}

template <typename PointT, typename TypeName>
std::vector<TypeName> KDTree<PointT, TypeName>::search(
    PointT &data, float distance_tolerance) {
  std::vector<TypeName> ids{};
  my_search(data, distance_tolerance, root, 0, ids);
  return ids;
}
