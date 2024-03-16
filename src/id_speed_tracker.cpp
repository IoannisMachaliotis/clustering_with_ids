// This version include some parts with structures, in my attempt to learn OOP. 

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>

// Project libraries
#include "clustering_with_ids/AEClustering.h"
#include "clustering_with_ids/Improve.h"
#include "clustering_with_ids/Tracking.h"
#include "clustering_with_ids/TerminalInfo.h"
#include "clustering_with_ids/Visualizer.h"

// --------Libraries added---------
#include <vector>
#include <math.h>
#include <iostream>

MatrixXd object_coordinates(2, 2);

using namespace Eigen;

unsigned int ID;
static std::vector<std::vector<double>> cluster_centers; // extracted list

// ----------------------------------

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;

// ---------- CLUSTERS PROCESS FUNCTIONS ------------
[[nodiscard]] std::vector<double> is_cluster_in(std::vector<double> &aVector, std::vector<std::vector<double>> &cluster_list)
{
    std::vector<double> Output;
    double IS_IN = 0;
    double x_v;
    double y_v;
    double timeOfNew;
    double InitializedMaxDist = 500;
    double IDx;
    double speed = 0;
    double dt,t_previous;

    int featureIterator = 0;
    // Get info from centersConverted
    for (double &aFeature : aVector)
    {
        if (featureIterator == 0)
        {
            x_v = aFeature;
        }
        if (featureIterator == 1)
        {
            y_v = aFeature;
        }
        if (featureIterator == 2)
        {
            timeOfNew = aFeature;
        }
        featureIterator++;
    }

    // Get info from cluster_centers
    for (const std::vector<double> &aVector : cluster_list)
    {
        double x;
        double y;
        double Id;
        double t_prev;

        int featureIteratorII = 0;
        for (const double &aCenter : aVector)
        {
            if (featureIteratorII == 0)
            {
                Id = aCenter;
            }
            if (featureIteratorII == 1)
            {
                x = aCenter;
            }
            if (featureIteratorII == 2)
            {
                y = aCenter;
            }
            if (featureIteratorII == 3)
            {
                t_prev = aCenter;
            }
            featureIteratorII++;
        }
        // Find the distance of each cluster from the new cluster
        const double distanceFromNew = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));

        if (distanceFromNew < InitializedMaxDist) // find the one who is closer
        {
            InitializedMaxDist = distanceFromNew;
            IDx = Id;
            t_previous = t_prev;
        }
    }
    if (InitializedMaxDist <= my_radius)    // assume it's the same cluster
    {
        IS_IN = IDx;   
        dt = timeOfNew - t_previous;        // Calculate time difference
        speed = InitializedMaxDist/dt;               // Calculate speed
    }
    else
    {                                       // assume it's a new one
        IS_IN = timeOfNew;
    }
    Output.push_back(IS_IN);        
    Output.push_back(speed);                // Add speed to list of vectors

    return Output;
}

void assigner(std::vector<std::vector<double>> &cluster_list, const std::vector<double> &centersConverted, const double &IS_IN)
{
    const double Is_in = IS_IN;
    std::vector<double> aTempVector;                     // vector -->  {}

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && cluster_list.empty())              // For the first iteration/cluster 
    {
        aTempVector.push_back(ID);                       // vector -->  {ID}
    }
    else if (Is_in > 100000)                             // if it's a timestamp
    {
        ID++;                                            // ----CREATE CLUSTER ID----
        aTempVector.push_back(ID);                       // vector -->  {ID}
    }
    else if (Is_in < 100000 && !cluster_list.empty()) // if it's an ID from condition: distance < limit
    {
        unsigned int indexOfClusterFound;
        // ---replace AFTER ERASING---
        unsigned int ClusterID = 0;
        for (const std::vector<double> &aVector : cluster_list)
        {
            if (aVector[0] == Is_in)
            {
                indexOfClusterFound = ClusterID;
            }
            ClusterID++;
        }
        cluster_list.erase(cluster_list.begin() + indexOfClusterFound);

        // ----ASSIGN SAME CLUSTER ID----
        aTempVector.push_back(Is_in);                    // vector -->  {ID}
    }
    for (const double &aCenter : centersConverted)
    {
        aTempVector.push_back(aCenter);                  // vector -->  {ID, x_new, y_new, t_stamp}
    }

    // Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
    cluster_list.push_back(aTempVector);

    // return cluster_list;
}


void clusters_assign_process(const VectorXd &cluster, MyCluster &ClusterCenters){
    double Is_in;

    // Convert cen vector from eigen vector to std::vector
    std::vector<double> cluster_converted(&cluster[0], cluster.data() + cluster.cols() * cluster.rows());

    // STORE THE TIMESTAMP ALONG SIDE WITH THE EVENT/CLUSTER
    ros::Time t_stamp = ros::Time::now();
    cluster_converted.push_back(t_stamp.toSec()); // Add the timestamp to new cluster

    // STORE THE # OF EVENTS WHICH GENERATED THIS CLUSTER
    cluster_converted.push_back(ClusterCenters.getN());

    // IS THE NEW CLUSTER ALREADY LISTED??
    const std::vector<double> updatedVector = is_cluster_in(cluster_converted, cluster_centers);

    unsigned int iter = 0;
    for ( const double &aFeature : updatedVector)
    {
        if (iter == 0)
        {
            Is_in = aFeature; // Get is_in value
        }
        if (iter == 1)
        {
          cluster_converted.push_back(aFeature); // add speed to data
        }
        iter++;
    }

    // ASSIGN THE NEW VECTOR TO cluster_centers
    assigner(cluster_centers, cluster_converted, Is_in);
}

std::vector<std::vector<double>> removal_KF_visualize(std::vector<std::vector<double>> &cluster_list, cv::Mat img)
{
    Improve* opt;
    Tracking* tracking;
    TerminalInfo* infoT;
    Visualizer* vis;
    // REMOVAL OF CLUSTERS THAT HAVE STOPPED BEING TRACKED
    opt->remover(cluster_list);

    if (terminal)
    {
        std::cout << "\n\n----- 🟣 ORIGINAL CLUSTERS ----\n";
    }

    // KALMAN FILTERING --->> improve clusters by precision
    if (!cluster_list.empty())
    {
        // SORT before using kalman filter
        sort(cluster_list.begin(), cluster_list.end());
        opt->kalmanfilter(cluster_list, kalman_centers);

        // Terminal View of kalman_centers
        if (terminal)
        {
            std::cout << "\n----- 🟠 KALMAN CENTERS ------\n";
            infoT->show_clusters(kalman_centers);
        }
    }
    else
    {
        kalman_centers.erase(kalman_centers.begin(), kalman_centers.end());
    }

    if (!kalman_centers.empty())
    {
        tracking->object_tracker(kalman_centers);
    }

    // TERMINAL VIEW of list created
    if (terminal)
    {
        std::cout << "\n----- 🟢 CLUSTERS LIST CREATED --\n";
        infoT->show_clusters(cluster_list);
    }

    // ---- VISUALIZATION OF CLUSTERS CREATED ----
    vis->visualizer(kalman_centers, cluster_list, object_coordinates, img); // orange and green

    // USE THE NEW KALMAN CENTERS for next iteration (FEEDBACK)
    cluster_list.erase(cluster_list.begin(), cluster_list.end());
    for (const std::vector<double> aVector : kalman_centers)
    {
        cluster_list.push_back(aVector);
    }

    delete opt;
    delete tracking;
    delete infoT;
    delete vis;

    return kalman_centers;
}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
    std::deque<double> ev(4, 0.0);
    for (const auto &e : msg->events)
    {
        ev[0] = e.ts.toSec();
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity;

        // Exclude broken pixel(231,202) due to hardware flaw of davis used at the lab
        if ((e.x > 231 || e.x < 229) & (e.y > 203 || e.y < 200))
        { 
            eclustering->update(ev);
        }
    }    

    int minN = eclustering->getMinN(); // 18 (take a look at the .launch file)
    for (auto &ClusterCenters : eclustering->clusters)
    {
        if (ClusterCenters.getN() >= minN) // if the cluster contains more than 15 events
        {
            Eigen::VectorXd cen(ClusterCenters.getClusterCentroid());

                if (!sort_by_events)
                {
                    // original clusters (without IDs)
                    cv::circle(im2, cv::Point(cen[0], cen[1]), 3, cv::Scalar(200, 0, 200), -1, 16); // purple

                    // ------------ radius ----------
                    if (radius_visual)
                    {
                        cv::circle(im2, cv::Point(cen[0], cen[1]), radius, cv::Scalar(0, 110, 0), 0, 16); // dark green
                    }
                }

                // Assign cluster
                clusters_assign_process(cen, ClusterCenters);
        }
    }

    // Remove clusters who stopped being tracked, apply Kalman Filter and visualize results
    removal_KF_visualize(cluster_centers, im2);

    // PUBLISH
    sensor_msgs::ImagePtr im_msg2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im2).toImageMsg();
    pubIm.publish(im_msg2);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        im2 = cv_bridge::toCvCopy(msg, "rgb8")->image; // Probably im2 is my visualized added data
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh("~");

    double alpha;
    int minN;
    double kappa;

    nh.getParam("szBuffer", szBuffer);
    nh.getParam("radius", radius);
    nh.getParam("kappa", kappa);
    nh.getParam("alpha", alpha);
    nh.getParam("minN", minN);

    eclustering->init(szBuffer, radius, kappa, alpha, minN);
    
    cv::Mat im2;

    ros::Subscriber subEv = nh.subscribe(
        "/dvs/events", 1, );            // Where to subscribe?

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("tracker_with_ids_image", 1, eventCallback);

    // auto imageCallback = [im2](const sensor_msgs::ImageConstPtr &msg)
    // {
    //     try
    //     {
    //         im2 = cv_bridge::toCvCopy(msg, "rgb8")->image; // Probably im2 is my visualized added data
    //         cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
    //     }
    //     catch (cv_bridge::Exception &e)
    //     {
    //         ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    //     }
    // };

    image_transport::Subscriber subIm = it.subscribe("dvs_rendering", 1, imageCallback); // Publish simultaniously with

    ros::spin();
}