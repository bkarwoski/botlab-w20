#ifndef NODE_HPP
#define NODE_HPP

#include <lcmtypes/pose_xyt_t.hpp>

class Node
{
    public:
    Node(pose_xyt_t); //setting up start node
    Node(pose_xyt_t, Node&); //making new node w/ parent node
    friend bool operator <(const Node& lhs, const Node& rhs);
    friend bool operator >(const Node& lhs, const Node& rhs);
    Node *parent;
    double gCost;
    double hCost;
    double dCost;
    pose_xyt_t pose;
    int x(void) const {return pose.x;}
    int y(void) const {return pose.y;}

};
#endif // NODE_HPP
