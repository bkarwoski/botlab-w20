#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <queue>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

struct node
{
    node *parent;
    pose_xyt_t pose;
    double cost;
    double hCost;
    double dCost
};

bool operator < (const node &n_lhs, const node &n_rhs){
    return (n_lhs.hCost + n_lhs.cost) < (n_rhs.hCost + n_rhs.cost);
}

bool operator > (const node &n_lhs, const node &n_rhs){
    return (n_lhs.hCost + n_lhs.cost) > (n_rhs.hCost + n_rhs.cost);
}

bool at_goal(pose_xyt_t goal, node pos);
void expand(node node);
double hCost(pose_xyt_t goal, pose_xyt_t pos);

/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);


std::priority_queue<node> open_list;
std::vector<node> closed_list;
#endif // PLANNING_ASTAR_HPP
