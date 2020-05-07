#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    for(std::size_t y = 0; y < map.heightInCells(); ++y)
    {
        for(std::size_t x = 0; x < map.widthInCells(); ++x)
        {
            if(map(x, y) < 0){
                //likely free
                distance(x,y) = 0;
            }
            if(map(x, y) >= 0){
                //likely occupied, or unknown
                distance(x, y) = 1;
            }
        }
    }
    for(std::size_t y = 0; y < map.heightInCells(); ++y){
        for(std::size_t x = 0; x < map.widthInCells(); ++x){
            if(distance(x, y) == 0){
                if(isCellInGrid(x+1, y) && distance(x+1, y) == 1){
                    distance(x,y) = 0.05;
                }
                else if(isCellInGrid(x, y+1) && distance(x, y+1) == 1){
                    distance(x,y) = 0.05;
                }
                else if(isCellInGrid(x-1, y) && distance(x-1, y) == 1){
                    distance(x,y) = 0.05;
                }
                else if(isCellInGrid(x, y-1) && distance(x, y-1) == 1){
                    distance(x,y) = 0.05;
                }
            }
        }
    }
    // double maxDist = 0.25;
    // for(double dist = 0.05; dist < maxDist; dist += 0.05){
    //     for(std::size_t y = 0; y < map.heightInCells(); ++y){
    //         for(std::size_t x = 0; x < map.widthInCells(); ++x){
    //             if(distance(x, y) == 0){
    //                     // std::cout << "found empty cell" << std::endl;
    //                 if(isCellInGrid(x+1, y) && fabs(distance(x+1, y) - dist) < __FLT_EPSILON__)
    //                 {
    //                     distance(x,y) = dist + 0.05;
    //                     std::cout << "to the right" << std::endl;
    //                 }
    //                 else if(isCellInGrid(x, y+1) && fabs(distance(x, y+1) - dist) < __FLT_EPSILON__){
    //                     distance(x,y) = dist + 0.05;
    //                 }
    //                 else if(isCellInGrid(x-1, y) && fabs(distance(x-1, y) - dist) < __FLT_EPSILON__){
    //                     distance(x,y) = dist + 0.05;
    //                 }
    //                 else if(isCellInGrid(x, y-1) && fabs(distance(x, y-1) - dist) < __FLT_EPSILON__){
    //                     distance(x,y) = dist + 0.05;
    //                 }
    //             }
    //         }
    //     }
    // }
    // copy over log odd from map directly
    // TODO: setting distances more intelligently (brushfire algo?)
}

bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}

void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
