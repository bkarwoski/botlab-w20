#ifndef NODE
#define NODE

#include <lcmtypes/pose_xyt_t.hpp>


class Node
{
    public:

    Node(pose_xyt_t); //setting up start node
    Node(pose_xyt_t, const Node); //making new node w/ parent node
    bool operator <(const Node& rhs);

    private:
    pose_xyt_t pose_;
    Node *parent_;




};
#endif // NODE