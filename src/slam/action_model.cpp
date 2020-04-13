#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void):
    alpha1_(0.01f),
    alpha2_(0.0001f),
    alpha3_(0.0025f),
    alpha4_(0.0001f),
    initialized_(false)
    {
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_)
    {
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX        = odometry.x - previousOdometry_.x;
    float deltaY        = odometry.y - previousOdometry_.y;
    float deltaTheta    = angle_diff(odometry.theta, previousOdometry_.theta);
    float direction = 1.0;

    trans_ = std::sqrt(deltaY * deltaY + deltaX * deltaX);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);

    if(std::abs(trans_) < 0.0001)
    {
        rot1_ = 0.0f;
    }
    
    else if(std::abs(rot1_) > M_PI / 2.0) {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);
    rot1Var_ = alpha1_ * rot1_ * rot1_ + alpha2_ * trans_ * trans_;
    rot2Var_ = alpha1_ * rot1_ * rot1_ + alpha2_ * trans_ * trans_;
    transVar_ = alpha3_ * trans_ * trans_ + alpha4_ * (rot1_ * rot1_ + rot2_ * rot2_);
    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    
    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    if(moved_){
        particle_t newSample = sample;
        float sampledRot1 = std::normal_distribution<>(rot1_, rot1Var_)(numberGenerator_);
        float sampledTrans = std::normal_distribution<>(trans_, transVar_)(numberGenerator_);
        float sampledRot2 = std::normal_distribution<>(rot2_, rot2Var_)(numberGenerator_);

        newSample.pose.x += sampledTrans * cos(sample.pose.theta + sampledRot1);
        newSample.pose.y += sampledTrans * sin(sample.pose.theta + sampledRot1);
        newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
        newSample.pose.utime = utime_;
        newSample.parent_pose = sample.pose;
        return newSample;
    }
    return sample;
}
