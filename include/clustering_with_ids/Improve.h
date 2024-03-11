#ifndef IMPROVE_H
#define IMPROVE_H
#pragma once 

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

using namespace Eigen;

// -------------- KALMAN FILTER VARIABLES ----------------
std::vector<std::vector<double>> kalman_centers; // Predicted centers

struct Improve
{
    std::vector<std::vector<double> >& remover(std::vector<std::vector<double> > &cluster_list);
    std::vector<std::vector<double> >& kalmanfilter(std::vector<std::vector<double> > &cluster_list, std::vector<std::vector<double> > &kalman_centers);
};
#endif