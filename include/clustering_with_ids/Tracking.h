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

// ----------- For tracking --------
const unsigned int speed_limit = 30;
const unsigned int  num_of_obj = 2;
MatrixXd object_coordinates(2,2);

struct Tracking
{
        VectorXd& track_by_most_events(const vector<vector<double>> &n, VectorXd &moving_obj);
        MatrixXd& object_tracker(const vector<vector<double>> &kalman_centers);
};