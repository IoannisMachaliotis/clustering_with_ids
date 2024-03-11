#ifndef VISUALIZER_H
#define VISUALIZER_H
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

cv::Mat im2;

double radius;
int szBuffer;

// Assumption that the one moving generates the most events
const bool sort_by_events = false;

const double my_radius = 15;

const bool clusters_list_created_visual = true;
const bool radius_visual = false;
const bool screen_details = true;
const bool terminal = true;

struct Visualizer
{
    void visualizer(const std::vector<std::vector<double>> &kalman_centers, std::vector<std::vector<double>> &cluster_list, const Eigen::MatrixXd &object_coordinates);
};
#endif