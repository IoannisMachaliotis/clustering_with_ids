#ifndef TRACKING_H
#define TRACKING_H
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// ----------- For tracking --------
const unsigned int speed_limit = 30;
const unsigned int num_of_obj = 2;
struct Tracking
{
        Eigen::VectorXd track_by_most_events(const std::vector<std::vector<double>> &kalman_centers, Eigen::VectorXd &moving_obj);
        Eigen::MatrixXd object_tracker(const std::vector<std::vector<double>> &kalman_centers);
};
#endif