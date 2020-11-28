#include <queue>
#include <planning/node.hpp>

class PriorityQueue
{
    public:
    std::priority_queue<Node> elements;
    bool isMember(int x, int y);
}