#include <planning/node.hpp>

Node::Node(pose_xyt_t pose)
: pose(pose)
, parent(NULL)
, gCost(0)
, hCost(0)
, dCost(0)
{
}

Node::Node(pose_xyt_t pose, Node &p)
: pose(pose)
, parent(&p)
{
}

bool operator < (const Node& lhs, const Node& rhs){
    return (lhs.hCost + lhs.gCost) < (rhs.hCost + rhs.gCost);
}

bool operator > (const Node& lhs, const Node& rhs){
    return (lhs.hCost + lhs.gCost) > (rhs.hCost + rhs.gCost);
}
