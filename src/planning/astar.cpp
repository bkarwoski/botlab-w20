#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/node.hpp>


std::priority_queue<Node> open_list;
std::vector<Node> closed_list;

bool at_goal(pose_xyt_t goal, Node node){
    return goal.x == node.x() && goal.y == node.y();
}

void expand(Node node){
//add the (up to) four nodes around the given node to the open list
}

double hCost(pose_xyt_t goal, Node node){
    // L2 Distance
    //return sqrt(pow(goal.x - pos.x, 2) + pow(goal.y - pos.y, 2));
    // L1 Distance
    return 10 * (abs(goal.x - node.x()) + abs(goal.y - node.y()));
}

double dCost(const SearchParams& params, pose_xyt_t pos) {
    //todo after obstacle_distance_grid actually works
    return 0;
}

double gCost(Node node){
    //can be expanded for 8-way search
    return node.parent->gCost + 10;
}

bool is_free(Node node, const ObstacleDistanceGrid& distances){
    if(distances(node.x(), node.y()) == 0) return true;
    return false;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    //if you're at the goal, construct the path and return it
    // Node currNode(start);
    // currNode.dCost = 0; //TODO dCost(&params, currNode.pose);
    // std::vector<node> closed_list;
    // std::priority_queue<node> open_list;
    Node_list closedNodes();
    robot_path_t path;
    path.utime = start.utime;
    //push back all the previous nodes, while node->parent isn't null
    // while(currNode.parent != NULL){
    //     path.path.push_back(currNode.pose);
    //     currNode = *currNode.parent;
    // }
    path.path.push_back(start);
    path.path_length = path.path.size();
    return path;
}
