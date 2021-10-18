#include <ros/callback_queue.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "grvc_e_clustering/AEClustering.h"
#include <vector>
#include <math.h>
#include <sstream>
#include <typeinfo>
#include <tuple>

using namespace std;

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;

std::vector<std::vector<double>> cluster_centers;

void show_vector(std::vector<double> n)
{
    int k = 0;
    for (double i : n)
    {
        std::cout << i;
        std::cout << " ,";
        k++;
    }
    std::cout << "\n";
}

void show_clusters(std::vector<std::vector<double>> cluster_centers){
    int Id = 0; // Create Cluster ID
    for (std::vector<double> n : cluster_centers){
        //show ID and center of new clusters
        std::cout << "ID:" << Id << "   Center: ";

        int k = 0;
        for (double i : n)
        {
            std::cout << i;
            if (k!=2){std::cout << ",  ";}
            k++;
        }
        std::cout << "\n";
        Id++;
    }
    std::cout << "\n";
}

int is_cluster_in(std::vector<double> v, std::vector<std::vector<double>> cluster_centers){

    double radius = 20 ;
    double IS_IN=0;
    int k = 0;
    double x_v;
    double y_v;
    double t_v;
    double d = 500;
    int ID=0;

    int j = 0;

    for (double i : v)
    {
        if (k == 0){x_v = i;}
        if (k == 1){y_v = i;}
        if (k == 2){t_v = i;}
        k++;
    }
    for (std::vector<double> n : cluster_centers)
    {
        int kk = 0;
        double x;
        double y;

        for (double ii : n){
            if (kk == 0){x = ii;}
            if (kk == 1){y = ii;}
            kk++;
        }
        //Find the distance of each cluster from the new cluster
        double d1 = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));
        if (d1 <= d) {
            d=d1;
            ID=j;
        }
        if (d < radius){
            IS_IN = ID;
        }else{
            IS_IN = t_v;
        }
        j++;
    }
    return IS_IN;
}

void visualizer(std::vector<std::vector<double>> n, cv::Mat im, double IS_IN)
{   
    double Is_in=IS_IN;
    double t_stamp;
    int j = 0;
    for (std::vector<double> v : n){
        cv::circle(im, cv::Point(v[0] + 2, v[1] + 2), 2, cv::Scalar(255, 0, 0), -1);

        std::string str;
        std::stringstream ss;
        ss << j;
        ss >> str;
        cv::putText(im, str, cv::Point(v[0] + 4, v[1] + 4), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 150, 200), 0, false);
        j++;
    }
}

vector<vector<double>> &remove_and_visualize(std::vector<std::vector<double>> &n, double Is_in , cv::Mat im){
    
    // ----REMOVAL---
    double buff_limit = 0.01; // seconds
    int ID = 0;
    double duration=0;
    for (std::vector<double> v: n){
        int k=0;
        for (double n : v){
            if (k==2) {
                ros::Time end = ros::Time::now();
                duration = end.toSec() - n;
            }
            k++;
        }
        if (duration > buff_limit){
            n.erase(n.begin()+ID); // Problem with erasing?? 
        }
        ID++;
    }

    // VISUALIZATION
    visualizer(n, im, Is_in);
    return n;
}
vector<vector<double>> &assigner( std::vector<std::vector<double>> &n, std::vector<double> cen_con, double IS_IN){
    double Is_in = IS_IN; 

    //CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && n.empty()) //For the first iteration/cluster
    {
        //Push back new cluster center, generated with ID, in a VECTOR
        n.push_back(cen_con);
    }
    else if ( Is_in < 40 && !n.empty() ) //For when we get an ID from condition: distance < limit
    {
        //Assign new cluster with the same Id
        n.erase(n.cbegin() + Is_in);
        auto Iterator = n.cbegin();
        n.insert(Iterator + Is_in, cen_con);
    }
    else if (Is_in > 40)// Then it's a timestamp 
    {
        //Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
        n.push_back(cen_con);
    }
    return n;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg){
    try
    {
        double Is_in;
        int minN = eclustering->getMinN(); // 15
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
        
        std::cout << "-----------NEW CLUSTERS----------\n";            

        for (auto cc : eclustering->clusters){
            Eigen::VectorXd cen(cc.getClusterCentroid());

            if (cc.getN() >= minN){ //if the clusters contains more than 15 events??
                
                cv::circle(im, cv::Point(cen[0], cen[1]), 2, cv::Scalar(0, 255, 0, 255), -1);

                // Convert cen vector from eigen vector to std::vector
                std::vector<double> cen_converted(&cen[0], cen.data() + cen.cols() * cen.rows());

                // STORE THE TIMESTAMP ALONG SIDE WITH THE EVENT/CLUSTER
                ros::Time t_stamp = ros::Time::now();
                cen_converted.push_back(t_stamp.toSec()); //Add the timestamp to new cluster

                // IS THE NEW CLUSTER ALREADY LISTED?? 
                Is_in = is_cluster_in(cen_converted, cluster_centers);

                // ASSIGN THE NEW VECTOR TO cluster_centers
                assigner(cluster_centers, cen_converted, Is_in);

                // REMOVAL OF CLUSTERS STOPPED BEING TRACKED
                remove_and_visualize(cluster_centers, Is_in, im);
            }
        }
        // TERMINAL VIEW
        show_clusters(cluster_centers);
        
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
    std::deque<double> ev(4, 0.0);
    for (const auto e : msg->events)
    {
        ev[0] = e.ts.toSec();
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity;
        eclustering->update(ev);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh("~");

    int szBuffer;
    double radius;
    double alpha;
    int minN;
    int kappa;

    nh.getParam("szBuffer", szBuffer);
    nh.getParam("radius", radius);
    nh.getParam("kappa", kappa);
    nh.getParam("alpha", alpha);
    nh.getParam("minN", minN);

    eclustering->init(szBuffer, radius, kappa, alpha, minN);

    ros::Subscriber subEv = nh.subscribe("/dvs/events", 1, eventCallback);

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("clusters_image", 1);
    image_transport::Subscriber subIm = it.subscribe("dvs/image_raw", 1, imageCallback);

    ros::spin();
}


// -------NOTES---------
/*
    *I dont create ID list, I only show and visualize with an order of IDs.
*/
