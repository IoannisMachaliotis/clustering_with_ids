#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "grvc_e_clustering/AEClustering.h"

// --------Libraries added---------
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <tuple>


using namespace std;

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;

vector<vector<double>> cluster_centers;
double ID;

void show_vector(vector<double> n) // For debugging purposes
{
    int k = 0;
    for (double i : n)
    {
        cout << i;
        cout << " ,";
        k++;
    }
    cout << "\n";
}

void show_clusters(vector<vector<double>> cluster_centers){
    //show ID and center of new clusters
    for (vector<double> n : cluster_centers){
        int k = 0;
        for (double i : n)
        {   
            if (k<=2 || k ==4){
                if (k == 0){cout << "ID:";}
                if (k == 1){cout << "x:";}
                if (k == 2){cout << "y:";}
                if (k == 3){cout << "t:";}
                if (k == 4){cout << "#ev:";}
                cout << i;
                if (k!=4){
                    if (k == 0){cout << "  ";}
                    else {cout << ",  ";}
                }
            }
            k++;
        }
    cout << "\n";
    }
}

int is_cluster_in(vector<double> v, vector<vector<double>> cluster_centers){

    double radius = 10;
    double IS_IN = 0;
    double x_v;
    double y_v;
    double t_v;
    double d = 500;
    double IDx;

    int k = 0;    
    // Get info from cen_con
    for (double i : v)
    {
        if (k == 0){x_v = i;}
        if (k == 1){y_v = i;}
        if (k == 2){t_v = i;}
        k++;
    }
    for (vector<double> n : cluster_centers){
        double x;
        double y;
        double Id;

        int kk = 0;
        for (double ii : n){
            if (kk == 0){Id = ii;}
            if (kk == 1){x = ii;}
            if (kk == 2){y = ii;}
            kk++;
        }
        //Find the distance of each cluster from the new cluster
        double d1 = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));
        if (d1 < d) {
            d = d1;
            IDx = Id;
        }
        if (d <= radius){
            IS_IN = IDx;
        }else{
            IS_IN = t_v;
        }
    }
    return IS_IN;
}

void visualizer(vector<vector<double>> n, cv::Mat im)
{   
    for (vector<double> v : n){
        cv::circle(im, cv::Point(v[1] + 2, v[2] + 2), 2, cv::Scalar(255, 150, 0), -1);

        string str;
        stringstream ss;
        ss << v[0]; //j
        ss >> str;
        cv::putText(im, str, cv::Point(v[1] + 4, v[2] + 4), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 150, 200), 0, false);
    }
}

vector<vector<double>> &remover(vector<vector<double>> &cluster_centers)
{
    double buff_limit = 0.0005; // seconds
    double Index;
    double ID;
    double duration = 0;
    int j = 0;
    for (vector<double> v : cluster_centers){
        int k = 0;
        for (double i : v){
            if (k == 0) {ID = i;}
            if (k == 3) {
                ros::Time end = ros::Time::now();
                duration = end.toSec() - i;
            }
            k++;
        }
        if (duration > buff_limit){
            Index = j;
            cluster_centers.erase(cluster_centers.begin()+Index);
        }
        j++;
    }
    return cluster_centers;
}
vector<vector<double>> &assigner( vector<vector<double>> &n, vector<double> cen_con, double IS_IN){
    double Is_in = IS_IN;
    vector<double> temp;            // vector -->  {} 

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && n.empty()){ //For the first iteration/cluster
        temp.push_back(ID);         // vector -->  {ID}
    }
    else if (Is_in > 100000){ //if it's a timestamp
        // ----CREATE CLUSTER ID----
        ID++;
        temp.push_back(ID);         // vector -->  {ID}
    }
    else if ( Is_in < 100000 && !n.empty() ){ // if it's an ID from condition: distance < limit
        int index;
        // ---replace AFTER ERASING---
        int j = 0;
        for (vector<double> v : n){
            if (v[0] == Is_in){index = j;}
            j++;
        }
        n.erase(n.begin()+ index);

        // ----ASSIGN SAME CLUSTER ID----
        temp.push_back(Is_in);         // vector -->  {ID}
    }
    for (double i : cen_con){
        temp.push_back(i);             // vector -->  {ID, x, y, t_stamp}
    }
    
    // Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
    n.push_back(temp);

    return n;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg){
    try{
        double Is_in;
        int minN = eclustering->getMinN(); // 15 (take a look at the .launch file)
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
        
        cout << "\n\n-----------NEW CLUSTERS----------\n";            

        for (auto cc : eclustering->clusters){
            if (cc.getN() >= minN){ //if the cluster contains more than 15 events
                Eigen::VectorXd cen(cc.getClusterCentroid());
                cv::circle(im, cv::Point(cen[0], cen[1]), 2, cv::Scalar(0, 255, 0, 255), -1);

                // Convert cen vector from eigen vector to std::vector
                vector<double> cen_converted(&cen[0], cen.data() + cen.cols() * cen.rows());

                // STORE THE TIMESTAMP ALONG SIDE WITH THE EVENT/CLUSTER
                ros::Time t_stamp = ros::Time::now();
                cen_converted.push_back(t_stamp.toSec()); //Add the timestamp to new cluster

                // STORE THE # OF EVENTS WHICH GENERATED THIS CLUSTER
                cen_converted.push_back(cc.getN());

                // IS THE NEW CLUSTER ALREADY LISTED?? 
                Is_in = is_cluster_in(cen_converted, cluster_centers);

                // ASSIGN THE NEW VECTOR TO cluster_centers
                assigner(cluster_centers, cen_converted, Is_in);
            }
        }

        // REMOVAL OF CLUSTERS STOPPED BEING TRACKED
        remover(cluster_centers);

        // VISUALIZATION
        visualizer(cluster_centers, im);

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
    deque<double> ev(4, 0.0);
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
    image_transport::Subscriber subIm = it.subscribe("dvs_rendering", 1, imageCallback);

    ros::spin();
}

// -------NOTES---------
/*
    if run -->> oo
        1) lagging (memory issues) from original program ---> improved
        2) run out of IDs (numbers)
        3) 1 cluster get stuck in the middle of the image 
*/
