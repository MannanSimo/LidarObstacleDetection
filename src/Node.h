#ifndef NODE_H_
#define NODE_H_

#include <memory>

template <typename PointT, typename Name>

struct Node {
  std::shared_ptr<PointT> data;
  Name id;
  std::shared_ptr<Node> left;
  std::shared_ptr<Node> right;

  Node(PointT &data_val, Name id_val);

  ~Node();

  void display();
};

#endif
