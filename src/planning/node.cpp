#include <planning/node.hpp>

Node::Node(pose_xyt_t pose)
: pose_(pose)
, parent_(NULL)
{
}


bool operator < (const Node& lhs, const Node& rhs){
    return (lhs.hCost_ + lhs.gCost_) < (rhs.hCost_ + rhs.gCost_);
}

bool operator > (const Node& lhs, const Node& rhs){
    return (lhs.hCost_ + lhs.gCost_) > (rhs.hCost_ + rhs.gCost_);
}
