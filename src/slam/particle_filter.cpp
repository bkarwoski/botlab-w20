#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>

#include <slam/sensor_model.hpp>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    std::random_device rd;
    std::mt19937 generator(rd()); //could be seeded with a const
    // std::normal_distribution<> dist(0.0, 0.01);
    // std::normal_distribution<> dist(0.0, 0.01);
    std::normal_distribution<> dist(0.0, 0.01); //x, y, theta

    for(auto& p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = posteriorPose_.theta + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.parent_pose = p.pose;
        p.weight = sampleWeight;

    }
    posterior_.back().pose = pose;
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}

particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}

std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    //Using the low variance resampling algorithm from Probabalistic Robotics

    std::vector<particle_t> prior;
    int M = posterior_.size();
    const double MInv = 1.0 / M;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, MInv);
    const double r = dis(gen);
    double c = posterior_[0].weight;
    int i = 0;
    double U = 0;
    for(int m = 0; m < M; m++){
        U = r + m * MInv;
        while(U > c){
            i++;
            c = c + posterior_[i].weight;
        }
        prior.push_back(posterior_[i]); //is weight normalized to 1?
    }
    return prior;
}

std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{   
    std::vector<particle_t> posterior;
    double normSum = 0;
    double likelihood = 0;
    for(int i = 0; i < proposal.size(); i++){
        likelihood = sensorModel_.likelihood(proposal[i], laser, map);
        normSum += likelihood;
        posterior.push_back(proposal[i]);
        posterior[i].weight = likelihood;
    }

    for(int i = 0; i < posterior.size(); i++){
        posterior[i].weight /= normSum;
    }
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution

    double xSum = 0;
    double ySum = 0;
    double cosSum = 0;
    double sinSum = 0;
    pose_xyt_t pose;

    for(int i = 0; i < posterior.size(); i++) {
        double w = posterior[i].weight;
        xSum += posterior[i].pose.x * w;
        ySum += posterior[i].pose.y * w;
        cosSum += cos(posterior[i].pose.theta) * w;
        sinSum += sin(posterior[i].pose.theta) * w;
    }
    double theta = atan2(sinSum, cosSum);
    pose.x = xSum;
    pose.y = ySum;
    pose.theta = theta;
    return pose;
}
