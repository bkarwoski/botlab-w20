#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>

inline double cell_likelihood(Point<int> cell, const OccupancyGrid& map)
{
    CellOdds odds = map.logOdds(cell.x, cell.y);
    return odds;
}

SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
     ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    
    //refer to Optimal Lidar Configuration doc for optimal values for each lidar
    const float kMaxLaserDistance = 10.0f;
    const float kMinRayLength = 0.2f;
    const float kSigmaLaser = 0.05;
    
    double scanLikelihood = 0.0;
    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    
    for(const auto& ray : movingScan)
    {
        if((ray.range < kMaxLaserDistance) && (ray.range > kMinRayLength))
        {
            double rayCost = 0.0;
            Point<double> endpoint(ray.origin.x + (ray.range * std::cos(ray.theta)), 
                                   ray.origin.y + (ray.range * std::sin(ray.theta)));
            auto rayEnd = global_position_to_grid_position(endpoint, map);
            rayCost = cell_likelihood(rayEnd, map);
            
            if(rayCost < 0) // if there wasn't a hit, then consider that the ray is a little short of what's expected
            {
                Point<double> nextEndpoint(rayEnd.x + std::cos(ray.theta), rayEnd.y + std::sin(ray.theta));
                rayCost = kSigmaLaser * cell_likelihood(nextEndpoint, map);
            }
            
            if(rayCost < 0) // if that didn't hit either, see if the ray estimate was a little longer than expected
            {
                Point<double> prevEndpoint(rayEnd.x - std::cos(ray.theta), rayEnd.y - std::sin(ray.theta));
                rayCost = kSigmaLaser * cell_likelihood(prevEndpoint, map);
            }

            // a ray cost of 0 is very unlikely, but is negative infinite log-likelihood. However, it isn't entirely
            // unreasonable, so penalize it without completely discounting the associated particle 
            scanLikelihood += rayCost;
        }
    }
    
    return scanLikelihood;
}
