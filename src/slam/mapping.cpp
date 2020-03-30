#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    //loop to iterate through lidar_t scans
        //check if ranges >= maxLaserDistance. If true, do nothing
        //Use pose to get starting cell
        //Use pose + scan range & bearing to get ending cell
        //For ending cell, add hitOdds to the cell's current CellOdds using setLogOdds
        //Use Breshenham function to get all empty cells
            //for each empty cell, increment logodds by missOdds
}
