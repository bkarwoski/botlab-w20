#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


bool at_goal(pose_xyt_t goal, node pos){
    return goal.x == pos.pose.x && goal.y == pos.pose.y;
}

void expand(node node){
//add the (up to) four nodes around the given node to the open list
}

double hCost(pose_xyt_t goal, pose_xyt_t pos){
    // L2 Distance
    //return sqrt(pow(goal.x - pos.x, 2) + pow(goal.y - pos.y, 2));
    // L1 Distance
    return 10 * (abs(goal.x - pos.x) + abs(goal.y - pos.y));
}

double dCost(SearchParams& const params, pose_xyt_t pos) {
    //todo after obstacle_distance_grid actually works
    return 0;
}

double gCost(node node){
    //can be expanded for 8-way search
    return node.parent->gCost + 10;
}

bool is_free(node node, const ObstacleDistanceGrid& distances){
    if(distances(node.pose.x, node.pose.y) == 0) return true;
    return false;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    //if you're at the goal, construct the path and return it
    node currNode;
    currNode.pose = start;
    currNode.gCost = 0;
    currNode.hCost = hCost(goal, start);
    currNode.dCost = 0; //TODO dCost(&params, currNode.pose);
    currNode.parent = NULL;
    // std::vector<node> closed_list;
    // std::priority_queue<node> open_list;

    while (!at_goal(goal, currNode) && )
    {
        /* code */
    }
    
    robot_path_t path;
    path.utime = start.utime;
    //push back all the previous nodes, while node->parent isn't null
    while(currNode.parent != NULL){
        path.path.push_back(currNode.pose);
        currNode = *currNode.parent;
    }
    path.path.push_back(start);
    path.path_length = path.path.size();
    return path;
}
