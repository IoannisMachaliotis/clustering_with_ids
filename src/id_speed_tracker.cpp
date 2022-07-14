// This version include some parts with structures, in my attempt to learn OOP. 

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "clustering_with_ids/AEClustering.h"
#include "clustering_with_ids/Improve.h"
#include "clustering_with_ids/Tracking.h"

// --------Libraries added---------
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;
using namespace Eigen;

double ID;
double my_radius = 15;
double radius;
int szBuffer;
static vector<vector<double>> cluster_centers; // extracted list

// ----- Visualization settings -----
bool clusters_list_created_visual = true;
bool radius_visual = false;
bool screen_details = true;
bool terminal = true;
// Assumption that the one moving generates the most events
bool sort_by_events = false;
double percentage_of_filtering = 0.2; // value range[0,1] lower-->more sensitive

// ----------------------------------

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;


// ----------- FUNCTIONS FOR DEBUGGING -----------------
void show_clusters(vector<vector<double>> cluster_list){
    // show ID and center of new clusters
    for (vector<double> n : cluster_list){
        int k = 0;
        for (double i : n){
            if (k <= 2 /*|| k ==4*/ || k == 5){
                if (k == 0){
                    cout << "ID:";
                }
                if (k == 1){
                    cout << "x:";
                }
                if (k == 2){
                    cout << "y:";
                }
                if (k == 3){
                    cout << "t:";
                }
                if (k == 4){
                    cout << "#ev:";
                }
                if (k == 5){
                    cout << "speed:";
                }
                cout << i;
                if (k != 5){
                    if (k == 0){
                        cout << "  ";
                    }
                    else{
                        cout << ",  ";
                    }
                }
            }
            k++;
        }
        cout << "\n";
    }
}

void show_vector(vector<double> n){
    int k = 0;
    for (double i : n){
        cout << i;
        cout << ", ";
        k++;
    }
    cout << "\n";
}
void show_centers(vector<vector<double>> cluster_centers){
    //show ID and center of new clusters
    for (vector<double> n : cluster_centers){
        int k = 0;
        for (double i : n){   
            if (k == 0){cout << "x:";}
            if (k == 1){cout << "y:";}
            cout << " " << i;
            if (k!=1){
                if (k == 0){cout << "  ";}
                else {cout << ",  ";}
            }
            k++;
        }
    cout << "\n";
    }
}
// ------------------------------------------------------


void visualizer(vector<vector<double>> &kalman_centers, vector<vector<double>> &cluster_centers, MatrixXd &object_coordinates){
    Tracking* tracking;
    double x1, y1, x2, y2;

    if (clusters_list_created_visual){
        for (vector<double> v : cluster_centers){                                                                             // kalman_centers with IDs
            cv::circle(im2, cv::Point(v[1], v[2]), 2, cv::Scalar(0, 255, 0), -1, 16); // orange
        }
    }

    if (sort_by_events){
        VectorXd moving_obj(3);
        tracking->track_by_most_events(kalman_centers, moving_obj);

        int most_events = moving_obj(2);
        if (most_events > percentage_of_filtering * szBuffer){                                                                        // filter out non moving objects
            cv::Point pt1(moving_obj(0) - radius, moving_obj(1) - (radius + 5)); // its top left corner...
            cv::Point pt2(moving_obj(0) + radius, moving_obj(1) + (radius + 5)); // its bottom right corner.
            cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0));                 // green
        }
    }
    else{

        for (vector<double> v : kalman_centers){                                                                               // kalman_centers with IDs
            cv::circle(im2, cv::Point(v[1], v[2]), 2, cv::Scalar(255, 150, 0), -1, 16); // orange

            // -------- my_radius --------
            if (radius_visual){
                cv::circle(im2, cv::Point(v[1], v[2]), my_radius, cv::Scalar(100, 0, 100), 0, 16); // dark purple
            }

            int id = v[0];
            int ev_num = v[4];
            string s1 = "ID:" + to_string(id);
            string s2 = "#ev:" + to_string(ev_num);

            cv::putText(im2, s1, cv::Point(v[1] + 4, v[2] + 4), cv::FONT_ITALIC, 0.3, cv::Scalar(100, 150, 200), 0, 16);
            cv::putText(im2, s2, cv::Point(v[1] + 4, v[2] - 6), cv::FONT_ITALIC, 0.3, cv::Scalar(255, 80, 80), 0, 16);
        }

        // ------------ Object Tracking --------------
        x1 = object_coordinates(0,0);
        x2 = object_coordinates(1,0);
        y1 = object_coordinates(0,1);
        y2 = object_coordinates(1,1);

        if (x1>0.1 &&  y1>0.1){
            cv::circle(im2, cv::Point(x1, y1), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
        }
        if (x2>0.1 && y2>0.1){
            cv::circle(im2, cv::Point(x2, y2), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
        }

        string s3 = "Number of objects being detected: " + to_string(num_of_obj);
        cv::putText(im2, s3, cv::Point(10, 37), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 0, 0), 0, 16); //red 

        delete tracking;
    }

    if (screen_details){
        // In case of sorting by events method
        if (sort_by_events){
            cv::Point pt1(184, 5);
            cv::Point pt2(188, 11);
            cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0));
            cv::putText(im2, "Moving Object", cv::Point(190, 11), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
            int ss = percentage_of_filtering * 100;
            string s3 = "Percentage of filtering: " + to_string(ss) + "%";
            cv::putText(im2, s3, cv::Point(190, 20), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
        }
        else{
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

// ---------- CLUSTERS PROCESS FUNCTIONS ------------
vector<double> is_cluster_in(vector<double> v, vector<vector<double>> cluster_centers) {
    vector<double> Output;
    double IS_IN = 0;
    double x_v;
    double y_v;
    double t_new;
    double d = 500;
    double IDx;
    double speed = 0;
    double dt,t_previous;

    int k = 0;
    // Get info from cen_con
    for (double i : v){
        if (k == 0){
            x_v = i;
        }
        if (k == 1){
            y_v = i;
        }
        if (k == 2){
            t_new = i;
        }
        k++;
    }

    // Get info from cluster_centers
    for (vector<double> n : cluster_centers){
        double x;
        double y;
        double Id;
        double t_prev;

        int kk = 0;
        for (double ii : n){
            if (kk == 0){
                Id = ii;
            }
            if (kk == 1){
                x = ii;
            }
            if (kk == 2){
                y = ii;
            }
            if (kk == 3){
                t_prev = ii;
            }
            kk++;
        }
        // Find the distance of each cluster from the new cluster
        double d1 = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));

        if (d1 < d){                 // find the one who is closer
            d = d1;
            IDx = Id;
            t_previous = t_prev;
        }
    }
    if (d <= my_radius){            // assume it's the same cluster
        IS_IN = IDx;   
        dt = t_new - t_previous;    // Calculate time difference
        speed = d/dt;               // Calculate speed
    }
    else{                           // assume it's a new one
        IS_IN = t_new;
    }
    Output.push_back(IS_IN);        // Add speed to list of vectors
    Output.push_back(speed);

    return Output;
}

vector<vector<double>>& assigner(vector<vector<double>>& n, vector<double> cen_con, double IS_IN){
    double Is_in = IS_IN;
    vector<double> temp;                    // vector -->  {}

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && n.empty()){           // For the first iteration/cluster                  
        temp.push_back(ID);                 // vector -->  {ID}
    }
    else if (Is_in > 100000){               // if it's a timestamp
        ID++;                               // ----CREATE CLUSTER ID----
        temp.push_back(ID);                 // vector -->  {ID}
    }
    else if (Is_in < 100000 && !n.empty()){ // if it's an ID from condition: distance < limit
        int index;
        // ---replace AFTER ERASING---
        int j = 0;
        for (vector<double> v : n){
            if (v[0] == Is_in){
                index = j;
            }
            j++;
        }
        n.erase(n.begin() + index);

        // ----ASSIGN SAME CLUSTER ID----
        temp.push_back(Is_in);              // vector -->  {ID}
    }
    for (double i : cen_con){
        temp.push_back(i);                  // vector -->  {ID, x_new, y_new, t_stamp}
    }

    // Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
    n.push_back(temp);

    return n;
}


vector<vector<double>>& clusters_assign_process(VectorXd& cluster, MyCluster cc){
    vector<double> INFO;
    double Is_in;

    // Convert cen vector from eigen vector to std::vector
    vector<double> cluster_converted(&cluster[0], cluster.data() + cluster.cols() * cluster.rows());

    // STORE THE TIMESTAMP ALONG SIDE WITH THE EVENT/CLUSTER
    ros::Time t_stamp = ros::Time::now();
    cluster_converted.push_back(t_stamp.toSec()); // Add the timestamp to new cluster

    // STORE THE # OF EVENTS WHICH GENERATED THIS CLUSTER
    cluster_converted.push_back(cc.getN());

    // IS THE NEW CLUSTER ALREADY LISTED??
    INFO = is_cluster_in(cluster_converted, cluster_centers);

    int i = 0;
    for ( double n : INFO){
        if (i == 0){ Is_in = n;} //Get is_in value
        if (i == 1){ cluster_converted.push_back(n);} // add speed to data
        i++;
    }

    // ASSIGN THE NEW VECTOR TO cluster_centers
    assigner(cluster_centers, cluster_converted, Is_in);

    return cluster_centers;
}

vector<vector<double>>& removal_KF_visualize(vector<vector<double>>& cluster_centers){
    Improve* opt;
    Tracking* tracking;
    // REMOVAL OF CLUSTERS STOPPED BEING TRACKED
    opt->remover(cluster_centers);

    if (terminal){
        cout << "\n\n----- 🟣 ORIGINAL CLUSTERS ----\n";
    }

    // KALMAN FILTERING --->> improve clusters by precision
    if (!cluster_centers.empty()){
        // SORT before using kalman filter
        sort(cluster_centers.begin(), cluster_centers.end());
        opt->kalmanfilter(cluster_centers, kalman_centers);

        // Terminal View of kalman_centers
        if (terminal){
            cout << "\n----- 🟠 KALMAN CENTERS ------\n";
            show_clusters(kalman_centers);
        }
    }
    else{
        kalman_centers.erase(kalman_centers.begin(), kalman_centers.end());
    }

    if (!kalman_centers.empty()){
        tracking->object_tracker(kalman_centers);
    }

    // TERMINAL VIEW of list created
    if (terminal){
        cout << "\n----- 🟢 CLUSTERS LIST CREATED --\n";
        show_clusters(cluster_centers);
    }

    // ---- VISUALIZATION OF CLUSTERS CREATED ----
    visualizer(kalman_centers, cluster_centers, object_coordinates); // orange and green

    // USE THE NEW KALMAN CENTERS for next iteration (FEEDBACK)
    cluster_centers.erase(cluster_centers.begin(), cluster_centers.end());
    for (vector<double> v : kalman_centers){
        cluster_centers.push_back(v);
    }

    delete opt;
    delete tracking;

    return kalman_centers;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        im2 = cv_bridge::toCvCopy(msg, "rgb8")->image;
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
    }
    catch (cv_bridge::Exception &e){
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}


void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg){
    std_msgs::Header h;

    deque<double> ev(4, 0.0);
    for (const auto e : msg->events){
        ev[0] = e.ts.toSec();
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity;

        if (e.x != 231 & e.y != 202){ // Exclude broken pixel(231,202) due to hardware flaw of davis used at the lab
            eclustering->update(ev);
        }
    }    

    int minN = eclustering->getMinN(); // 18 (take a look at the .launch file)
    for (auto cc : eclustering->clusters){
        if (cc.getN() >= minN){ // if the cluster contains more than 15 events
            Eigen::VectorXd cen(cc.getClusterCentroid());

                if (!sort_by_events){
                    // original clusters (without IDs)
                    cv::circle(im2, cv::Point(cen[0], cen[1]), 3, cv::Scalar(200, 0, 200), -1, 16); // purple

                    // ------------ radius ----------
                    if (radius_visual){
                        cv::circle(im2, cv::Point(cen[0], cen[1]), radius, cv::Scalar(0, 110, 0), 0, 16); // dark green
                    }
                }

                // Assign cluster
                clusters_assign_process(cen, cc);
        }
    }

    // Remove clusters who stopped being tracked, apply Kalman Filter and visualize results
    removal_KF_visualize(cluster_centers);

    // PUBLISH
    sensor_msgs::ImagePtr im_msg2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im2).toImageMsg();
    pubIm.publish(im_msg2);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh("~");

    double alpha;
    int minN;
    int kappa;

    nh.getParam("szBuffer", szBuffer);
    nh.getParam("radius", radius);
    nh.getParam("kappa", kappa);
    nh.getParam("alpha", alpha);
    nh.getParam("minN", minN);

    eclustering->init(szBuffer, radius, kappa, alpha, minN);

    ros::Subscriber subEv = nh.subscribe("/dvs/events", 1, eventCallback);              // Where to subscribe?

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("tracker_with_ids_image", 1);
    image_transport::Subscriber subIm = it.subscribe("dvs_rendering", 1, imageCallback); // Publish simultaniously with

    ros::spin();
}