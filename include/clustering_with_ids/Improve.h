#pragma once 
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace Eigen;
cv::Mat im2;

// -------------- KALMAN FILTER VARIABLES ----------------
std::vector<std::vector<double>> kalman_centers; // Predicted centers

struct Improve {
    std::vector<std::vector<double> >& remover(std::vector<std::vector<double> > &cluster_list);
    std::vector<std::vector<double> >& kalmanfilter(std::vector<std::vector<double> > &cluster_list, std::vector<std::vector<double> > &kalman_centers);
};