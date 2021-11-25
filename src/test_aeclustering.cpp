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
#include <deque>

#define SIZE 10      // (Memory)Size of History
#define CLUSTERS 15  // Enough for calculating at any moment

using namespace std;
using namespace Eigen;

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;
cv::Mat im2;

vector<vector<double>> cluster_centers;
double ID;

// -------------- KALMAN FILTER VARIABLES ----------------
vector<vector<double>> kalman_centers; // Predicted centers
int n = 3;                             // Number of states (position, velocity, acceleration)
int m = 1;                             // Number of measurements

double ddt;
VectorXd dt(CLUSTERS,1);               // Time step
MatrixXd A(n,n);                       // State Matrix
MatrixXd C(m, n);                      // Output matrix
MatrixXd Q(n, n);                      // Process noise covariance (keeps the state covariance matrix from becoming too small, or going to 0)
MatrixXd R(m, m);                      // Measurement covariance matrix (Error in the measurement)
MatrixXd P(n, n);                      // State covariance matrix (Error in the estimate)
MatrixXd K;                            // Kalman Gain (based on comparing the error in the estimate to the error in the measurement)
MatrixXd I(n,n);                       // Identity Matrix 
VectorXd x_hat(m,n), x_hat_new(m,n);   // Estimated states
VectorXd x0(n);
VectorXd v_temp(1,2);
VectorXd y(m);
// -------- For History -----------
MatrixXd x_history(CLUSTERS,SIZE);
MatrixXd y_history(CLUSTERS,SIZE);
VectorXd col_i;
VectorXd col_j;
VectorXd coli;
VectorXd colj;
VectorXd sum_x(CLUSTERS,1);
VectorXd sum_y(CLUSTERS,1);
VectorXd x_mean(CLUSTERS,1);
VectorXd y_mean(CLUSTERS,1);
MatrixXd Mean(CLUSTERS,2); //Merging
VectorXd deviation_x(CLUSTERS,1);
VectorXd deviation_y(CLUSTERS,1);
MatrixXd deviation(CLUSTERS,2);//Merging
VectorXd variance_x(CLUSTERS,1);
VectorXd variance_y(CLUSTERS,1);
MatrixXd M_variance(CLUSTERS,2);//Merging

// ----------- FUNCTIONS FOR DEBUGGING -----------------
void show_vector(vector<double> n) 
{
    int k = 0;
    for (double i : n)
    {
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
        for (double i : n)
        {   
            if (k == 0){cout << "x:";}
            if (k == 1){cout << "y:";}
            cout << i;
            if (k!=1){
                if (k == 0){cout << "  ";}
                else {cout << ",  ";}
            }
            k++;
        }
    cout << "\n";
    }
}
void show_clusters(vector<vector<double>> cluster_centers){
    //show ID and center of new clusters
    for (vector<double> n : cluster_centers){
        int k = 0;
        for (double i : n)
        {   
            if (k<=2 /*|| k ==4*/){
                if (k == 0){cout << "ID:";}
                if (k == 1){cout << "x:";}
                if (k == 2){cout << "y:";}
                if (k == 3){cout << "t:";}
//                if (k == 4){cout << "#ev:";}
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

// ------------------------------------------------------




void KF_visualizer(vector<vector<double>> KF_n,  cv::Mat im)
{
    for (vector<double> v : KF_n){
        cv::circle(im, cv::Point(v[0], v[1]), 2, cv::Scalar(0, 255, 0), -1);
    }
}

void visualizer(vector<vector<double>> n, cv::Mat im)
{   
    for (vector<double> v : n){
        cv::circle(im, cv::Point(v[1],  v[2]), 2, cv::Scalar(255, 150, 0), -1);

        string str;
        stringstream ss;
        ss << v[0];
        ss >> str;
        cv::putText(im, str, cv::Point(v[1] + 4, v[2] + 4), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 150, 200), 0, false);
    }
}

VectorXd KF_algorithm(VectorXd& y)
{
//    cout << "\nA:" << A << "\nx_hat_new:"<< x_hat_new << "\nx_hat:" << x_hat ;
    x_hat_new = A * x_hat;
//    cout << "\nx_hat_new:" << x_hat_new << "\nP:" << P << "\nQ:"<< Q;
    P = A*P*(A.transpose()) + Q;
//    cout << "\nP:" << P << "\nK:" << K << "\nC:"<< C;
    K = (P*C.transpose())*(C*P*(C.transpose()) + R).inverse();
//    cout << "\nK:" << K << "\ny:" << y << "\nR:"<< R;
    x_hat_new += K*(y - C*x_hat_new );
//    cout << "\nx_hat_new:"<< x_hat_new;
    P = (I - K*C)*P;
    x_hat = x_hat_new;

    return x_hat;
}

vector<vector<double>> &kalmanfilter(vector<vector<double>> &cluster_centers, vector<vector<double>> &kalman_centers, ros::Time start)
{
    //Erase for refresh list
    kalman_centers.erase(kalman_centers.begin(), kalman_centers.end()); 

    ros::Time end = ros::Time::now();
//    ddt = (end - start).toSec();

    // ------------- 2D IMPLEMENTATION of State Space Kalman Filter ---------------
    vector<vector<double>> position_vector = {};
    vector<double> temporary_IDs = {};
    // Initialize dt vector for each cluster
    for (int i = 0; i < CLUSTERS; i++){
        dt(i) = 0;
    }

    // ------- OBTAIN X, Y, and dt FROM cluster_centers --------
    int counter1 = 0;
    for (vector<double> v : cluster_centers){
        vector<double> temp = {};
        int k = 0;
        for (double i:v){
            if (k == 0){temporary_IDs.push_back(i);}
            if (k == 1 || k == 2){ // 2D(x&y_axis_position)
                temp.push_back(i);
            }

            if (k == 3){ // Obtain dt for each cluster
                dt(counter1) = end.toSec() - i;
            }

            k++;
        }
        position_vector.push_back(temp);
        counter1++;
    }

    // CONVERT BACK TO EIGEN MATRICES and Create History
    MatrixXd POS_MEAS_MAT_2D(position_vector.size(),2);
    int counter2 = 0;
    for (vector<double> v:position_vector){
        int j = 0;
        for (double A:v){
            POS_MEAS_MAT_2D(counter2,j) = A;

            // for creating temporary HISTORY
            if (j == 1){
                y_history(counter2) = A;
            }
            if (j == 0){
                x_history(counter2) = A;
            }

            j++;
        }
        counter2++;
    }

    // ------------------- HISTORY CREATION ------------------    

    // SWAP columns to store the next measurement
    //y_history
    for (int ii = SIZE-1; ii >= 0; ii--){
        col_i = y_history.col(ii+1);
        col_j = y_history.col(ii);
        y_history.col(ii) = col_i;
        y_history.col(ii+1) = col_j;
    }
    //x_history
    for (int ii = SIZE-1; ii >= 0; ii--){
        coli = x_history.col(ii+1);
        colj = x_history.col(ii);
        x_history.col(ii) = coli;
        x_history.col(ii+1) = colj;
    }
    // Put zeros in 1st cols because of a bug (computer puts random small numbers instead)
    x_history.col(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    y_history.col(0) << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

    cout << "-------- HISTORY OF MEASUREMENTS -------\n";
    cout << "x_history  NEW ---->-->--> OLD\n";
    cout << x_history << "\n";
    cout << "y_history  NEW ---->-->--> OLD\n";
    cout << y_history << "\n";
 
    // Intializing Q, R, P
    for (int i = 0; i < CLUSTERS; i++){
        x_mean(i,1) = 0;
        y_mean(i,1) = 0;
        sum_x(i) = 0;
        sum_y(i) = 0;
        deviation_x(i) = 0;
        deviation_y(i) = 0;
        variance_x(i) = 0;
        variance_y(i) = 0;
    }

    // Calculating mean values of each cluster for the last HISTORY SIZE measurements
    double counter_x;
    double counter_y;
    for ( int i = 0; i < CLUSTERS; i++){
        counter_x = 0;
        counter_y = 0;
        for ( int j = 0; j < SIZE; j++){
            if (x_history(i,1) != 0){ // if the last measurement is there (for every existing cluster)
                if (x_history(i,j) != 0){
                    sum_x(i) += x_history(i,j);
                    counter_x++;
                }
            }
            if (y_history(i,1) != 0){ // if the last measurement is there (for every existing cluster)
                if (y_history(i,j) != 0){
                    sum_y(i) += y_history(i,j);
                    counter_y++;
                } 
            }
        }
        if (counter_x != 0){
            x_mean(i) = sum_x(i)/counter_x;
        }
        if (counter_y != 0){
            y_mean(i) = sum_y(i)/counter_y;
        }
    }
    Mean << x_mean, y_mean;
//    cout << "mean is:\n" << Mean << "\n";

    // Calculating MEASUREMENTS Deviation and Variance
    double counter_xx;
    double counter_yy;
    for ( int i = 0; i < CLUSTERS; i++){
        counter_xx = 0;
        counter_yy = 0;
        for ( int j = 1; j < SIZE; j++){
            if (x_history(i,1) != 0){ // if the last measurement is there (for every existing cluster)
                if (x_history(i,j) != 0){
                    deviation_x(i) += abs(Mean(i,j) - x_history(i,j));
                    counter_xx++;
                }
            }
            if (y_history(i,1) != 0){ // if the last measurement is there (for every existing cluster)
                if (y_history(i,j) != 0){
                    deviation_y(i) += abs(Mean(i,j) - y_history(i,j));
                    counter_yy++;
                }
            }
        }

        if (counter_xx != 0){
            deviation_x(i) = deviation_x(i)/(counter_xx);
            variance_x(i) = pow(deviation_x(i),2)/(counter_xx);
        }
        if (counter_yy != 0){
            deviation_y(i) = deviation_y(i)/(counter_yy);
            variance_y(i) = pow(deviation_y(i),2)/(counter_yy);
        }

    }

    // Merge vectors to Matrices
    deviation << deviation_x, deviation_y;
    M_variance << variance_x, variance_y;
//    cout << "deviation:\n" << deviation << "\n";
//    cout << "variance is:\n" << M_variance << "\n";
//    cout << "\ndt is:\n" << dt << "\n";

    // ----- ESTIMATE KALMAN CENTERS based on the info obtained ------
    if (POS_MEAS_MAT_2D.isZero() == 0) // If there is any cluster in Eigen Matrix
    {
        for (int i = 0; i < POS_MEAS_MAT_2D.rows(); i++)
        {
            Q << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
            R << 0;
            P << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
            vector<double> temp_updated_center = {}; // Create std::vector which will be pushed_back every 2 iterations
            for (int j = 0; j < POS_MEAS_MAT_2D.cols(); j++)
            {
                y << POS_MEAS_MAT_2D(i,j); // current measurement

                // Discrete LTI projectile motion, measuring position only
                A << 1, dt(i), 0,
                     0, 1,   dt(i),
                     0, 0,     1;                            // 3x3
                C << 1, 0, 0;                                // 3x1
                I.setIdentity();                             // Identity Matrix
                
                // Covariance matrices Based on my data
                Q << 0.5*pow(dt(i),2), 0.5*pow(dt(i),2), 0,
                     0.5*pow(dt(i),2), 0.5*pow(dt(i),2); 0,  // Mean of measurements covariance ()
                     0,             0,             0;
                R << M_variance(i,j);
                P << .1, .1, .1,                             // State covariance
                    .1, 10000, 10,                           // Maybe should be Gaussian noise
                    .1, 10, 100;                             // distrebuted around the measurement/state?

                // Best guess of initial states
                x0 << y, 3, 0;
                // INITIALIZE state
                x_hat = x0;
                // UPDATE MEASUREMENT
                v_temp << KF_algorithm(y);

                // Create kalman_centers list
                double ii = v_temp[0];
                temp_updated_center.push_back(ii);

                if (temp_updated_center.size() == 2){ //Every 2 iterations (x & y)
                    kalman_centers.push_back(temp_updated_center);
                    temp_updated_center.erase(temp_updated_center.begin(),temp_updated_center.end());
                }
            }
        }
    }
    
    return kalman_centers;
} 

int is_cluster_in(vector<double> v, vector<vector<double>> cluster_centers){

    double radius = 50;
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

vector<vector<double>> &remover(vector<vector<double>> &cluster_centers)
{
    double buff_limit = 0.1; // seconds
    double Index;
    double ID;
    double duration = 0;
    int j = 0;
    for (vector<double> v : cluster_centers){
        int k = 0;
        for (double i : v){
            if (k == 0) {ID = i;}
            if (k == 3) {
                ros::Time checkpoint = ros::Time::now();
                duration = checkpoint.toSec() - i;
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
    vector<double> temp;              // vector -->  {} 

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && n.empty()){ //For the first iteration/cluster
        temp.push_back(ID);           // vector -->  {ID}
    }
    else if (Is_in > 100000){ //if it's a timestamp
        // ----CREATE CLUSTER ID----
        ID++;
        temp.push_back(ID);           // vector -->  {ID}
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
        im2 = cv_bridge::toCvCopy(msg, "bgr8" )->image;
        cv::Mat im = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
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

    ros::Time start = ros::Time::now();
    double Is_in;
    int minN = eclustering->getMinN(); // 18 (take a look at the .launch file)

    for (auto cc : eclustering->clusters){
        if (cc.getN() >= minN){ //if the cluster contains more than 15 events
            Eigen::VectorXd cen(cc.getClusterCentroid());\
            // original clusters (without IDs)
            cv::circle(im2, cv::Point(cen[0], cen[1]), 2, cv::Scalar(200, 0, 200), -1); //purple

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

    cout << "\n---- 🟣 original clusters ----\n";

    // KALMAN FILTERING --->> improve clusters by precision
    if (!cluster_centers.empty()){
        // SORT before using kalman filter
        sort(cluster_centers.begin(),cluster_centers.end());
        kalmanfilter(cluster_centers, kalman_centers, start);
  
        // Terminal View of kalman_centers
        cout << "\n\n---- 🟢 kalman_centers ----\n";
        show_centers(kalman_centers);  
    }

    // TERMINAL VIEW of list created
    cout << "\n--- 🟠 clusters_list created ---\n";
    show_clusters(cluster_centers);

    // ---- VISUALIZATION OF CLUSTERS CREATED ----
    // kalman_centers
    KF_visualizer(kalman_centers, im2); //green
    // cluster_centers
    visualizer(cluster_centers, im2); //orange

    // PUBLISH
    sensor_msgs::ImagePtr im_msg2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im2).toImageMsg();
    pubIm.publish(im_msg2);
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
