#include "TerminalInfo.h"
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
// #include <print>

// ----------- FUNCTIONS FOR DEBUGGING -----------------
void TerminalInfo::show_clusters(const std::vector<std::vector<double>> &cluster_list)
{
  // show ID and center of new clusters
  for (const std::vector<double> &aCluster : cluster_list) 
  {
    std::size_t featureIterator = 0;
    for (const double &aVector : aCluster)
    {
      if (featureIterator <= 2 /*|| featureIterator ==4*/ ||
          featureIterator == 5) {
        if (featureIterator == 0)
        {
          std::cout << "ID:";
        }
        if (featureIterator == 1)
        {
          std::cout << "x:";
        }
        if (featureIterator == 2)
        {
          std::cout << "y:";
        }
        if (featureIterator == 3) 
        {
          std::cout << "t:";
        }
        if (featureIterator == 4)
        {
          std::cout << "#ev:";
        }
        if (featureIterator == 5)
        {
          std::cout << "speed:";
        }
        std::cout << aVector;
        if (featureIterator != 5) 
        {
          if (featureIterator == 0)
          {
            std::cout << "  ";
          }
          else
          {
            std::cout << ",  ";
          }
        }
      }
      featureIterator++;
    }
    std::cout << "\n";
  }
}

void TerminalInfo::show_vector(const std::vector<double> &aVector)
{
  std::size_t iterator = 0;
  for (const double &aCoordinate : aVector)
  {
    std::cout << aCoordinate;
    std::cout << ", ";
    iterator++;
  }
  std::cout << "\n";
}
void TerminalInfo::show_centers(const std::vector<std::vector<double>> &cluster_list)
{
  // show ID and center of new clusters
  for (const std::vector<double> &aVector : cluster_list)
  {
    std::size_t featureIterator = 0;
    for (const double &aCenter : aVector)
    {
      if (featureIterator == 0)
      {
        std::cout << "x:";
      }
      if (featureIterator == 1)
      {
        std::cout << "y:";
      }
      std::cout << " " << aCenter;
      if (featureIterator != 1)
      {
        if (featureIterator == 0)
        {
          std::cout << "  ";
        }
        else
        {
          std::cout << ",  ";
        }
      }
      featureIterator++;
    }
    std::cout << "\n";
  }
}
// ------------------------------------------------------
