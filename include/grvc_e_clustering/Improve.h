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
using namespace std;
cv::Mat im2;

// -------------- KALMAN FILTER VARIABLES ----------------
vector<vector<double>> kalman_centers; // Predicted centers

struct Improve {
    vector<vector<double> >& remover(vector<vector<double> >& cluster_centers);
    vector<vector<double> >& kalmanfilter(vector<vector<double> >& cluster_centers, vector<vector<double> >& kalman_centers);
};