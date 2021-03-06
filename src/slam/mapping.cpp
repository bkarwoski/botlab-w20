#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <numeric>

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
    : kMaxLaserDistance_(maxLaserDistance), kHitOdds_(hitOdds), kMissOdds_(missOdds), initialized_(false)
{
}

void Mapping::updateMap(const lidar_t &scan, const pose_xyt_t &pose, OccupancyGrid &map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    //loop to iterate through lidar_t scans
    //check if ranges >= maxLaserDistance. If true, do nothing
    //Use pose to get starting cell
    //Use pose + scan range & bearing to get ending cell
    //For ending cell, add hitOdds to the cell's current CellOdds using setLogOdds
    //Use Breshenham function to get all empty cells
    //for each empty cell, increment logodds by missOdds
    // for(auto& ray : scan()) {
    if (!initialized_)
    {
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for (auto &ray : movingScan)
    {
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }
    //todo: Breshenham
    //scoreRay(ray, map)

    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreRay(const adjusted_ray_t ray, OccupancyGrid &map)
{
    if (ray.range <= kMaxLaserDistance_)
    {
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayEnd;
        rayEnd.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayEnd.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        int dx = abs(rayEnd.x - rayStart.x);
        int dy = abs(rayEnd.y - rayStart.y);
        int signX = rayStart.x < rayEnd.x ? 1 : -1;
        int signY = rayStart.y < rayEnd.y ? 1 : -1;
        int err = dx - dy;
        int x = rayStart.x;
        int y = rayStart.y;
        //TODO: skip last point (actually, this is already done)
        while (x != rayEnd.x || y != rayEnd.y)
        {
            decreaseCellOdds(x, y, map);
            int err2 = 2 * err;
            if (err2 >= -dy)
            {
                err -= dy;
                x += signX;
            }
            if (err2 <= dx)
            {
                err += dx;
                y += signY;
            }
        }
    }
}

void Mapping::scoreEndpoint(const adjusted_ray_t ray, OccupancyGrid &map)
{
    if (ray.range <= kMaxLaserDistance_)
    {
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;
        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        if (map.isCellInGrid(rayCell.x, rayCell.y))
        {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid &map)
{
    if (!initialized_)
    {
        //nothing to do
    }

    else if (std::numeric_limits<CellOdds>::max() - map(x, y) > kHitOdds_)
    {
        map(x, y) += kHitOdds_;
    }

    else
    {
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid &map)
{
    if (!initialized_)
    {
        //nothing to do
    }

    else if (std::numeric_limits<CellOdds>::min() - map(x, y) < -1 * kMissOdds_)
    {
        map(x, y) -= kMissOdds_;
    }

    else
    {
        map(x, y) = std::numeric_limits<CellOdds>::min();
    }
}
