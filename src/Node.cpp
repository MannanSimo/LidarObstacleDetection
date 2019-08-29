#include "Node.h"
#include <utility>
#include <iostream>

template <typename PointT, typename Name>
Node<PointT, Name>::Node(PointT &data_val, Name id_val)
    : data{std::make_shared<PointT>(data_val)}, id{id_val}
{
    std::cout << "Node created: "<< id << std::endl;
}

template <typename PointT, typename Name>
Node<PointT, Name>::~Node()
{
    std::cout<< "Node destroyed: " << id << std::endl;
}

template <typename PointT, typename Name>
void Node<PointT, Name>::display()
{
    std::cout << "-----------------------" << std::endl;
    std::cout << "Displaying Info:" << std::endl;
    std::cout << "ID: " << id << std::endl;
    std::cout << "Data: ";
    for (const auto &d : *data)
    {
        std::cout << d << " ";
    }
    std::cout << std::endl;
    std::cout << "Left: " << left << std::endl;
    std::cout << "Right " << right << std::endl;
    std::cout << "-----------------------" << std::endl;
}
