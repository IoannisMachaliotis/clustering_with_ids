#include "Visualizer.h"
#include "Tracking.h"
#include <ros/ros.h>
#include <sstream>

// Assumption that the one moving generates the most events
const double percentage_of_filtering = 0.2; // value range[0,1] lower-->more sensitive

// ----- Visualization settings -----
extern const bool clusters_list_created_visual;
extern const bool radius_visual;
extern const bool screen_details;
extern const bool terminal;

using namespace Eigen;

void Visualizer::visualizer(
    const std::vector<std::vector<double>> &kalman_centers,
    std::vector<std::vector<double>> &cluster_list,
    const MatrixXd &objectCoordinates,
    cv::Mat im2)
    {
  Tracking *tracking;
  double x1, y1, x2, y2;

  if (clusters_list_created_visual)
  {
    for (const std::vector<double> &aVector : cluster_list)
    { // kalman_centers with IDs
      cv::circle(im2, cv::Point(aVector[1], aVector[2]), 2, cv::Scalar(0, 255, 0), -1, 16); // orange
    }
  }

  if (sort_by_events)
  {
    VectorXd moving_obj(3);
    tracking->track_by_most_events(kalman_centers, moving_obj);

    const unsigned int most_events = moving_obj(2);
    if (most_events > percentage_of_filtering * szBuffer)
    { // filter out non moving objects
      cv::Point pt1(moving_obj(0) - radius, moving_obj(1) - (radius + 5)); // its top left corner...
      cv::Point pt2(moving_obj(0) + radius, moving_obj(1) + (radius + 5)); // its bottom right corner.
      cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0)); // green
    }
  }
  else
  {
    for (const std::vector<double> &aVector : kalman_centers)  // kalman_centers with IDs
    {
      cv::circle(im2, cv::Point(aVector[1], aVector[2]), 2, cv::Scalar(255, 150, 0), -1, 16); // orange

      // -------- my_radius --------
      if (radius_visual)
      {
        cv::circle(im2, cv::Point(aVector[1], aVector[2]), my_radius, cv::Scalar(100, 0, 100), 0, 16); // dark purple
      }

      const int id = aVector[0];
      const int ev_num = aVector[4];
      std::string s1 = "ID:" + std::to_string(id);
      std::string s2 = "#ev:" + std::to_string(ev_num);

      cv::putText(im2, s1, cv::Point(aVector[1] + 4, aVector[2] + 4), cv::FONT_ITALIC, 0.3, cv::Scalar(100, 150, 200), 0, 16);
      cv::putText(im2, s2, cv::Point(aVector[1] + 4, aVector[2] - 6), cv::FONT_ITALIC, 0.3, cv::Scalar(255, 80, 80), 0, 16);
    }

    // ------------ Object Tracking --------------
    x1 = objectCoordinates(0, 0);
    x2 = objectCoordinates(1, 0);
    y1 = objectCoordinates(0, 1);
    y2 = objectCoordinates(1, 1);

    if (x1 > 0.1 && y1 > 0.1)
    {
      cv::circle(im2, cv::Point(x1, y1), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
    }
    if (x2 > 0.1 && y2 > 0.1)
    {
      cv::circle(im2, cv::Point(x2, y2), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
    }

    std::string s3 = "Number of objects being detected: " + std::to_string(num_of_obj);
    cv::putText(im2, s3, cv::Point(10, 37), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 0, 0), 0, 16); // red

    delete tracking;
  }

  if (screen_details)
  {
    // In case of sorting by events method
    if (sort_by_events)
    {
      cv::Point pt1(184, 5);
      cv::Point pt2(188, 11);
      cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0));
      cv::putText(im2, "Moving Object", cv::Point(190, 11), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
      const unsigned int ss = percentage_of_filtering * 100;
      std::string s3 = "Percentage of filtering: " + std::to_string(ss) + "%";
      cv::putText(im2, s3, cv::Point(190, 20), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
    }
    else
    {
      // ------ Clusters info -------
      cv::circle(im2, cv::Point(225, 7), 3, cv::Scalar(200, 0, 200), -1, 16); // purple
      cv::putText(im2, "Original clusters", cv::Point(230, 10), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
      cv::circle(im2, cv::Point(225, 17), 3, cv::Scalar(255, 150, 0), -1, 16); // orange
      cv::putText(im2, "Corrected centers", cv::Point(230, 19), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
      cv::circle(im2, cv::Point(225, 27), 3, cv::Scalar(0, 255, 0), -1, 16); // green
      cv::putText(im2, "Extracted centers", cv::Point(230, 29), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
    }

    // ------- Camera info -------
    cv::putText(im2, "Camera/Sensor: DAVIS 350x250", cv::Point(3, 10), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(80, 100, 185), 0, 16);
    cv::circle(im2, cv::Point(5, 17), 2, cv::Scalar(0, 0, 255), -1, 16); // blue
    cv::putText(im2, "Positive events", cv::Point(10, 19), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
    cv::circle(im2, cv::Point(5, 26), 2, cv::Scalar(255, 0, 0), -1, 16); // red
    cv::putText(im2, "Negative events", cv::Point(10, 28), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
  }
}