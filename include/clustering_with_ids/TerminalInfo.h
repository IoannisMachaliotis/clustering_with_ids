#pragma once

#include <vector>

struct TerminalInfo
{
    void show_clusters(const std::vector<std::vector<double>> &cluster_list);
    void show_vector(const std::vector<double> &aVector);
    void show_centers(const std::vector<std::vector<double>> &cluster_list);
};
