#ifndef NODE_HPP
#define NODE_HPP

#include <lcmtypes/pose_xyt_t.hpp>

class Node
{
    public:

    Node(pose_xyt_t); //setting up start node
    Node(pose_xyt_t, const Node); //making new node w/ parent node
    friend bool operator <(const Node& lhs, const Node& rhs);
    friend bool operator >(const Node& lhs, const Node& rhs);

    private:
    pose_xyt_t pose_;
    Node *parent_;
    double gCost_;
    double hCost_;
    double dCost_;

};
#endif // NODE_HPP
