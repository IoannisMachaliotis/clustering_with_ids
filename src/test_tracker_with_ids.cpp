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

#define CLUSTERS 15

using namespace std;
using namespace Eigen;

double ID;
double my_radius = 15;
double radius;
int szBuffer;
vector<vector<double>> cluster_centers; // extracted list

// ----- Visualization settings -----
bool clusters_list_created_visual = true;
bool radius_visual = false;
bool details = false;
bool terminal = true;
bool accuracy_visual = false;
// Assumption that the one moving generates the most events
bool sort_by_events = false;
double percentage_of_filtering = 0.2; // value range[0,1] lower-->more sensitive

// ----------------------------------

AEClustering *eclustering(new AEClustering);

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;
cv::Mat im2;

// -------------- KALMAN FILTER VARIABLES ----------------
vector<vector<double>> kalman_centers; // Predicted centers
int n = 3;                             // Number of states (position, velocity, acceleration)
int m = 1;                             // Number of measurements

double dt;
MatrixXd A(n, n);                      // State Matrix
MatrixXd C(m, n);                      // Output matrix
MatrixXd Q(n, n);                      // Process noise covariance (keeps the state covariance matrix from becoming too small, or going to 0)
MatrixXd R(m, m);                      // Measurement covariance matrix (Error in the measurement)
MatrixXd P(n, n);                      // State covariance matrix (Error in the estimate)
MatrixXd P_previous(n, n);
MatrixXd K;                            // Kalman Gain (based on comparing the error in the estimate to the error in the measurement)
MatrixXd I(n, n);                      // Identity Matrix
VectorXd x_hat(m, n), x_hat_new(m, n); // Estimated states
VectorXd x_hat_previous(m, n);
VectorXd x0(n);
VectorXd v_temp(3);
VectorXd y(m);
// ------ For last Kalman Centers (Feedback) ------
MatrixXd previous_KF_centers(CLUSTERS, 2);

// ------ For Accuracy -------
MatrixXd Extracted_cen(CLUSTERS, 2);   // Extracted Centers
MatrixXd Predicted_cen(CLUSTERS, 2);   // Predicted Centers
VectorXd Accuracy_mat(CLUSTERS, 1);    // Accuracy
MatrixXd Velocity_mat(CLUSTERS, 2);    // Velocity

// ----------- For tracking --------
double speed_limit = 30;
vector<vector<double>> speed_centers;
MatrixXd object_coordinates(2,2);
int num_of_obj = 2;

// ----------- FUNCTIONS FOR DEBUGGING -----------------
void show_clusters(vector<vector<double>> cluster_list)
{
    // show ID and center of new clusters
    for (vector<double> n : cluster_list)
    {
        int k = 0;
        for (double i : n)
        {
            if (k <= 2 /*|| k ==4*/ || k == 5)
            {
                if (k == 0)
                {
                    cout << "ID:";
                }
                if (k == 1)
                {
                    cout << "x:";
                }
                if (k == 2)
                {
                    cout << "y:";
                }
                if (k == 3)
                {
                    cout << "t:";
                }
                if (k == 4)
                {
                    cout << "#ev:";
                }
                if (k == 5){
                    cout << "speed:";
                }
                cout << i;
                if (k != 5)
                {
                    if (k == 0)
                    {
                        cout << "  ";
                    }
                    else
                    {
                        cout << ",  ";
                    }
                }
            }
            k++;
        }
        cout << "\n";
    }
}

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

VectorXd &track_by_most_events(vector<vector<double>> &n, VectorXd &moving_obj)
{
    double most_events = -1;
    for (vector<double> v : n)
    {
        bool flag = false;
        int k = 0;
        for (double i : v)
        {
            if (k == 4)
            {
                if (most_events < i)
                {
                    most_events = i;
                    flag = true;
                }
            }
            k++;
        }
        if (flag)
        {
            moving_obj << v[1], v[2], most_events; // x, y, #ev
        }
    }
    return moving_obj;
}

void visualizer(vector<vector<double>> &kalman_centers, vector<vector<double>> &cluster_centers, MatrixXd &object_coordinates)
{
    double x1, y1, x2, y2;

    if (clusters_list_created_visual)
    {
        for (vector<double> v : cluster_centers)
        {                                                                             // kalman_centers with IDs
            cv::circle(im2, cv::Point(v[1], v[2]), 2, cv::Scalar(0, 255, 0), -1, 16); // orange
        }
    }

    if (sort_by_events)
    {
        VectorXd moving_obj(3);
        track_by_most_events(kalman_centers, moving_obj);

        int most_events = moving_obj(2);
        if (most_events > percentage_of_filtering * szBuffer)
        {                                                                        // filter out non moving objects
            cv::Point pt1(moving_obj(0) - radius, moving_obj(1) - (radius + 5)); // its top left corner...
            cv::Point pt2(moving_obj(0) + radius, moving_obj(1) + (radius + 5)); // its bottom right corner.
            cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0));                 // green
        }
    }
    else
    {

        for (vector<double> v : kalman_centers)
        {                                                                               // kalman_centers with IDs
            cv::circle(im2, cv::Point(v[1], v[2]), 2, cv::Scalar(255, 150, 0), -1, 16); // orange

            // -------- my_radius --------
            if (radius_visual)
            {
                cv::circle(im2, cv::Point(v[1], v[2]), my_radius, cv::Scalar(100, 0, 100), 0, 16); // dark purple
            }

            int id = v[0];
            int ev_num = v[4];
            string s1 = "ID:" + to_string(id);
            string s2 = "#ev:" + to_string(ev_num);

            cv::putText(im2, s1, cv::Point(v[1] + 4, v[2] + 4), cv::FONT_ITALIC, 0.3, cv::Scalar(100, 150, 200), 0, 16);
            cv::putText(im2, s2, cv::Point(v[1] + 4, v[2] - 6), cv::FONT_ITALIC, 0.3, cv::Scalar(255, 80, 80), 0, 16);
        }
    }

    if (details)
    {
        // In case of sorting by events method
        if (sort_by_events)
        {
            cv::Point pt1(184, 5);
            cv::Point pt2(188, 11);
            cv::rectangle(im2, pt1, pt2, cv::Scalar(0, 255, 0));
            cv::putText(im2, "Moving Object", cv::Point(190, 11), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
            int ss = percentage_of_filtering * 100;
            string s3 = "Percentage of filtering: " + to_string(ss) + "%";
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
        cv::circle(im2, cv::Point(5, 17), 2, cv::Scalar(0, 0, 255), -1, 16); // red
        cv::putText(im2, "Positive events", cv::Point(10, 19), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
        cv::circle(im2, cv::Point(5, 26), 2, cv::Scalar(255, 0, 0), -1, 16); // blue
        cv::putText(im2, "Negative events", cv::Point(10, 28), cv::FONT_ITALIC, 0.3, cv::Scalar(200, 200, 0), 0, 16);
    }

    // ------------ Object Tracking --------------
    x1 = object_coordinates(0,0);
    x2 = object_coordinates(1,0);
    y1 = object_coordinates(0,1);
    y2 = object_coordinates(1,1);

//    cout << "x1:" << x1 << "  x2:" << x2 << "  y1:" << y1 << "  y2:" << y2;
    if (x1>0.1 &&  y1>0.1){
        cv::circle(im2, cv::Point(x1, y1), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
    }
    if (x2>0.1 && y2>0.1){
        cv::circle(im2, cv::Point(x2, y2), 10, cv::Scalar(255, 150, 0), 0, 16); // orange
    }

}


VectorXd KF_algorithm(VectorXd &y)
{
    x_hat_new = A * x_hat;
    P = A * P * (A.transpose()) + Q;
    K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse();
    P = (I - K * C) * P * (I - K * C).transpose() + K * R * K.transpose();
    x_hat_new += K * (y - C * x_hat_new);

    return x_hat_new;
}

VectorXd Eucleidian_acc(vector<vector<double>> &cluster_centers, vector<vector<double>> &kalman_centers)
{
    int size = cluster_centers.size();

    // Initialize Matrices
    for (int i = 0; i < CLUSTERS; i++)
    {
        for (int j = 0; j <= 1; j++)
        {
            Extracted_cen(i, j) = 0;
            Predicted_cen(i, j) = 0;
            Velocity_mat(i, j) = 0;
        }
        Accuracy_mat(i) = 0;
    }

    // Convert from std to Eigen vectors for processing
    int counter = 0;
    for (vector<double> v : cluster_centers)
    {
        int j = 0;
        for (double A : v)
        {
            if (j != 0)
            {
                Extracted_cen(counter, j - 1) = A;
            }
            j++;
        }
        counter++;
    }

    int counter1 = 0;
    for (vector<double> v : kalman_centers)
    {
        int j = 0;
        for (double A : v)
        {
            if (j != 0)
            {
                Predicted_cen(counter1, j - 1) = A;
            }
            j++;
        }
        counter1++;
    }

    // Calculate Accuracy
    double x_ex;
    double y_ex;
    double x_pr;
    double y_pr;
    double distance;
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            if (j == 0)
            {
                x_ex = Extracted_cen(i, j);
                x_pr = Predicted_cen(i, j);
            }
            if (j == 1)
            {
                y_ex = Extracted_cen(i, j);
                y_pr = Predicted_cen(i, j);
            }
        }
        // Eycleidian Distance between corrected and original
        distance = sqrt(pow(x_ex - x_pr, 2) + pow(y_ex - y_pr, 2));
        // Accuracy with reference to original clusters
        Accuracy_mat(i) = 1 - (distance) / sqrt(pow(x_pr, 2) + pow(y_pr, 2));
    }

    return Accuracy_mat;
}

vector<vector<double>> &kalmanfilter(vector<vector<double>> &cluster_centers, vector<vector<double>> &kalman_centers)
{
    dt = 20;

    // ------------- 2D IMPLEMENTATION of State Space Kalman Filter ---------------
    vector<vector<double>> position_vector = {};
    vector<double> temporary_IDs = {};
    vector<double> temporary_num_of_ev = {};
    vector<double> temporary_t_stamp = {};
    vector<double> temporary_speed = {};
    // ------- OBTAIN info FROM cluster_centers(to assign back afterwards) --------
    int counter1 = 0;
    for (vector<double> v : cluster_centers)
    {
        vector<double> temp_xy = {};
        int k = 0;
        for (double i : v)
        {
            if (k == 0)
            { // IDs
                temporary_IDs.push_back(i);
            }
            if (k == 1 || k == 2)
            { // 2D(x&y_axis_position)
                temp_xy.push_back(i);
            }
            if (k == 3)
            { // t_stamp
                temporary_t_stamp.push_back(i);
            }
            if (k == 4)
            { //# events
                temporary_num_of_ev.push_back(i);
            }
            if (k == 5)
            { // speed
                temporary_speed.push_back(i);
            }
            k++;
        }
        position_vector.push_back(temp_xy);
        counter1++;
    }

    // CONVERT BACK TO EIGEN MATRICES
    VectorXd temp_IDs(position_vector.size(), 1);        // IDs
    MatrixXd POS_MEAS_MAT_2D(position_vector.size(), 2); // Position
    VectorXd temp_t_stamp(position_vector.size(), 1);    // t_stamps
    VectorXd temp_num_ev(position_vector.size(), 1);     // # of events
    VectorXd temp_speed(position_vector.size(),1);       // speed

    int counter_1 = 0;
    for (double i : temporary_IDs)
    { // IDs
        temp_IDs(counter_1) = i;
        counter_1++;
    }
    int counter_2 = 0;
    for (vector<double> v : position_vector)
    { // Position
        int j = 0;
        for (double A : v)
        {
            POS_MEAS_MAT_2D(counter_2, j) = A;
            j++;
        }
        counter_2++;
    }
    int counter_3 = 0;
    for (double i : temporary_t_stamp)
    { // t_stamps
        temp_t_stamp(counter_3) = i;
        counter_3++;
    }
    int counter_4 = 0;
    for (double i : temporary_num_of_ev)
    { // # of events
        temp_num_ev(counter_4) = i;
        counter_4++;
    }
    int counter_5 = 0;
    for (double i : temporary_speed)
    { // speed
        temp_speed(counter_5) = i;
        counter_5++;
    }

    // Erase for refresh list
    kalman_centers.erase(kalman_centers.begin(), kalman_centers.end());

    // Discrete LTI projectile motion, measuring position only
    A << 1, dt, 0,
        0, 1, dt,
        0, 0, 1;     // 3x3
    C << 1, 1, 1;    // 3x1
    I.setIdentity(); // Identity Matrix

    // Covariance matrices Based on my data
    Q << 0.5 * pow(dt, 2), 0.5 * pow(dt, 2), .0,
        0.5 * pow(dt, 2), 0.5 * pow(dt, 2), .0, // Mean of measurements covariance
        .0, .0, .0;

    R << 1; // Observation Covariance
    P << .1, .1, .1,
        .1, 10000, 1, // Current State covariance
        .1, 1, 100;

    // ----- ESTIMATE KALMAN CENTERS based on the info obtained ------
    if (POS_MEAS_MAT_2D.isZero() == 0) // If there is any cluster in Eigen Matrix
    {
        for (int i = 0; i < POS_MEAS_MAT_2D.rows(); i++)
        {
            vector<double> temp_updated_center = {}; // Create std::vector which will be pushed_back every 2 iterations
            for (int j = 0; j < POS_MEAS_MAT_2D.cols(); j++)
            {
                y << POS_MEAS_MAT_2D(i, j); // current measurement

                // Best guess of initial states
                if (previous_KF_centers.row(i).isZero() == 1)
                {
                    x0 << y, 0, 0;
                }
                else
                {
                    x0 << previous_KF_centers(i, j), 0, 0;
                }
                // INITIALIZE state
                x_hat = x0;

                // UPDATE MEASUREMENT
                v_temp << KF_algorithm(y);

                // Create kalman_centers list
                double ii = v_temp(0);

                // Assign Velocity
                Velocity_mat(i, j) = v_temp(1);
                //                cout << Velocity_mat << "\n";

                if (j == 0)
                {
                    // Assign the ID back
                    temp_updated_center.push_back(temp_IDs(i)); // vector --> {ID}
                }
                temp_updated_center.push_back(ii); // vector --> {ID, x, y}
                if (j == 1)
                {
                    // Assign the timestamps back                  vector --> {ID, x, y, t_stamp}
                    temp_updated_center.push_back(temp_t_stamp(i));

                    // Assign the # of events back                 vector --> {ID, x, y, t_stamp, #ev}
                    temp_updated_center.push_back(temp_num_ev(i));

                    // Assign the speed back                       vector --> {ID, x, y, t_stamp, #ev, speed}
                    temp_updated_center.push_back(temp_speed(i));
                }

                if (temp_updated_center.size() == 6)
                { // Every 2 iterations (x & y)
                    kalman_centers.push_back(temp_updated_center);
                    temp_updated_center.erase(temp_updated_center.begin(), temp_updated_center.end());
                }
            }
        }
    }

    //    cout << "Velocity Matrix is:\n" << Velocity_mat << "\n";

    // Delete rows that do not exist in new kalman_centers
    int previous_KF_size = 0;
    for (int i = 0; i < previous_KF_centers.rows(); i++)
    {
        if (previous_KF_centers(i, 0) != 0)
        {
            previous_KF_size++;
        }
    }
    if (previous_KF_size > kalman_centers.size())
    {
        for (int i = kalman_centers.size(); i < previous_KF_centers.rows(); i++)
        {
            for (int j = 0; j <= 1; j++)
            {
                previous_KF_centers(i, j) = 0;
            }
        }
    }

    // CONVERT kalman_centers to EIGEN MATRICES and store for next iteration
    int counter3 = 0;
    for (vector<double> v : kalman_centers)
    {
        int j = 0;
        for (double A : v)
        {
            if (j != 0)
            {
                previous_KF_centers(counter3, j - 1) = A;
            }
            j++;
        }
        counter3++;
    }

    // Accuracy measurement
    Accuracy_mat << Eucleidian_acc(cluster_centers, kalman_centers);

    // Choose if you want accuracy visualized
    if (accuracy_visual)
    {
        int counter6 = 0;
        for (vector<double> v : kalman_centers)
        {

            int num = (1 - Accuracy_mat(counter6)) * 100;
            string s = "Impr:" + to_string(num) + "%";

            cv::putText(im2, s, cv::Point(v[1] + 4, v[2] - 14), cv::FONT_ITALIC, 0.3, cv::Scalar(50, 50, 255), 0, 16);

            counter6++;
        }
        cout << "Accuracy for each cluster is:\n"
             << Accuracy_mat << "\n";
    }

    return kalman_centers;
}

vector<double> is_cluster_in(vector<double> v, vector<vector<double>> cluster_centers)
{
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
    for (double i : v)
    {
        if (k == 0)
        {
            x_v = i;
        }
        if (k == 1)
        {
            y_v = i;
        }
        if (k == 2)
        {
            t_new = i;
        }
        k++;
    }

    // Get info from cluster_centers
    for (vector<double> n : cluster_centers)
    {
        double x;
        double y;
        double Id;
        double t_prev;

        int kk = 0;
        for (double ii : n)
        {
            if (kk == 0)
            {
                Id = ii;
            }
            if (kk == 1)
            {
                x = ii;
            }
            if (kk == 2)
            {
                y = ii;
            }
            if (kk == 3){
                t_prev = ii;
            }
            kk++;
        }
        // Find the distance of each cluster from the new cluster
        double d1 = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));

        if (d1 < d)                  // find the one who is closer
        {
            d = d1;
            IDx = Id;
            t_previous = t_prev;
        }
    }
    if (d <= my_radius)          // assume it's the same cluster
    {
        IS_IN = IDx;   
        dt = t_new - t_previous; // Calculate time difference
//        cout << "dt:" << dt << "\n";
        speed = d/dt;            // Calculate speed
//            cout << "speed\n";
    }
    else                         // assume it's a new one
    {
        IS_IN = t_new;
    }
    Output.push_back(IS_IN);         // Add speed to list of vectors
    Output.push_back(speed);
/*    
    cout << "output vector is:\n";
    show_vector(Output);
*/
    return Output;
}

vector<vector<double>> &remover(vector<vector<double>> &cluster_centers)
{
    double buff_limit = 0.01; // seconds (how fast to remove)
    double Index;
    double duration = 0;

    int counter = 0;
    for (vector<double> v : cluster_centers)
    {
        int k = 0;
        for (double i : v)
        {
            if (k == 3)
            {
                ros::Time checkpoint = ros::Time::now();
                duration = checkpoint.toSec() -  i;
            }
            k++;
        }
        if (duration > buff_limit)
        {
            Index = counter;
            cluster_centers.erase(cluster_centers.begin() + Index);
        }
        counter++;
    }
    return cluster_centers;
}
vector<vector<double>> &assigner(vector<vector<double>> &n, vector<double> cen_con, double IS_IN)
{
    double Is_in = IS_IN;
    vector<double> temp; // vector -->  {}

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && n.empty())
    {                       // For the first iteration/cluster
        temp.push_back(ID); // vector -->  {ID}
    }
    else if (Is_in > 100000)
    { // if it's a timestamp
        // ----CREATE CLUSTER ID----
        ID++;
        temp.push_back(ID); // vector -->  {ID}
    }
    else if (Is_in < 100000 && !n.empty())
    { // if it's an ID from condition: distance < limit
        int index;
        // ---replace AFTER ERASING---
        int j = 0;
        for (vector<double> v : n)
        {
            if (v[0] == Is_in)
            {
                index = j;
            }
            j++;
        }
        n.erase(n.begin() + index);

        // ----ASSIGN SAME CLUSTER ID----
        temp.push_back(Is_in); // vector -->  {ID}
    }
    for (double i : cen_con)
    {
        temp.push_back(i); // vector -->  {ID, x_new, y_new, t_stamp}
    }

    // Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
    n.push_back(temp);

    return n;
}

vector<vector<double>> &clusters_assign_process(VectorXd &cluster, MyCluster cc)
{
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

MatrixXd &object_tracker(vector<vector<double>> &kalman_centers)
{
    // ------------ Calculate speed among all clusters ---------------
    int size = kalman_centers.size();
    MatrixXd kalman_centers_mat(size,6);
    VectorXd last_col(size);
    VectorXd col_i(size);
    VectorXd col_j(size);
    vector<double> temp_data;
    double x1,y1,x2,y2, speed_of_cluster;

    // INITALIZE WITH ZEROS
    for (int i = 0; i <= size-1 ; i++)
    {
        for ( int j = 0; j <= 5; j++) // for all 6 data
        {
            kalman_centers_mat(i,j) = 0;
        }
        col_i(i) = 0;
        col_j(i) = 0;
        last_col(i) = 0;
    }
    
    // Convert to Eigen matrix
    int i = 0;
    for (vector<double> v : kalman_centers)
    { 
        int j = 0;
        for (double A : v)
        {
            kalman_centers_mat(i, j) = A;
            j++;
        }
        i++;
    }

    // SWAP LAST COLUMN WITH FIRST ONE
    last_col = kalman_centers_mat.col(5);
    for (int ii = 4; ii >= 0; ii--){ //size of data -2
        col_i = kalman_centers_mat.col(ii+1);
        col_j = kalman_centers_mat.col(ii);
        kalman_centers_mat.col(ii) = col_i;
        kalman_centers_mat.col(ii+1) = col_j;
    }
    kalman_centers_mat.col(0) = last_col;

    speed_centers.erase(speed_centers.begin(), speed_centers.end());
    // Convert from Eigen to std again
    for (int i = 0; i <= size-1 ; i++)
    {
        temp_data = {};
        for ( int j = 0; j <= 5; j++)
        {
            temp_data.push_back(kalman_centers_mat(i,j));
        }
        speed_centers.push_back(temp_data);
    }
    sort(speed_centers.begin(),speed_centers.end(), greater<vector<double>>());

    object_coordinates << 0, 0, 0, 0;
    // Get the first one(with higher speed) which must be the moving object
    int kk = 0;
    for ( vector<double> v : speed_centers)
    {
        int k = 0;
        if (kk <= num_of_obj-1){ // Specify the number of moving objects that you want to detect
            for ( double p : v)
            {
                if ( k == 0){ speed_of_cluster = p;} // Get the speed of each cluster
                if (speed_of_cluster > speed_limit){
                    if (kk==0){
                        if ( k == 2){x1 = p;}
                        if ( k == 3){y1 = p;}
                    }
                    if (kk==1){
                        if ( k == 2){x2 = p;}
                        if ( k == 3){y2 = p;}
                    }
                }
                k++;
            }
        }
        kk++;
    }
    
    object_coordinates << x1, y1,
                          x2, y2;
//    cout << "\nobject coordinates:\n";
//    cout << object_coordinates;
    return object_coordinates;
}

vector<vector<double>> &removal_KF_visualize(vector<vector<double>> &cluster_centers)
{
    // REMOVAL OF CLUSTERS STOPPED BEING TRACKED
    remover(cluster_centers);

    if (terminal)
    {
        cout << "\n\n----- 🟣 ORIGINAL CLUSTERS ----\n";
    }

    // KALMAN FILTERING --->> improve clusters by precision
    if (!cluster_centers.empty())
    {
        // SORT before using kalman filter
        sort(cluster_centers.begin(), cluster_centers.end());
        kalmanfilter(cluster_centers, kalman_centers);

        // Terminal View of kalman_centers
        if (terminal)
        {
            cout << "\n----- 🟠 KALMAN CENTERS ------\n";
            show_clusters(kalman_centers);
        }
    }
    else
    {
        kalman_centers.erase(kalman_centers.begin(), kalman_centers.end());
    }

    if (!kalman_centers.empty())
    {
        object_tracker(kalman_centers);
    }

    // TERMINAL VIEW of list created
    if (terminal)
    {
        cout << "\n----- 🟢 CLUSTERS LIST CREATED --\n";
        show_clusters(cluster_centers);
    }

    // ---- VISUALIZATION OF CLUSTERS CREATED ----
    visualizer(kalman_centers, cluster_centers, object_coordinates); // orange and green

    // USE THE NEW KALMAN CENTERS for next iteration (FEEDBACK)
    cluster_centers.erase(cluster_centers.begin(), cluster_centers.end());
    for (vector<double> v : kalman_centers)
    {
        cluster_centers.push_back(v);
    }

    return kalman_centers;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        im2 = cv_bridge::toCvCopy(msg, "rgb8")->image;
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
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

    int minN = eclustering->getMinN(); // 18 (take a look at the .launch file)
    for (auto cc : eclustering->clusters)
    {
        if (cc.getN() >= minN)
        { // if the cluster contains more than 15 events
            Eigen::VectorXd cen(cc.getClusterCentroid());

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
            clusters_assign_process(cen, cc);
        }
    }

    // Remove clusters who stopped being tracked, apply Kalman Filter and visualize results
    removal_KF_visualize(cluster_centers);

    // PUBLISH
    sensor_msgs::ImagePtr im_msg2 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im2).toImageMsg();
    pubIm.publish(im_msg2);
}

int main(int argc, char **argv)
{
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

    ros::Subscriber subEv = nh.subscribe("/feature_events", 1, eventCallback); // Where to subscribe?

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("clusters_image", 1);
    image_transport::Subscriber subIm = it.subscribe("dvs_rendering_corners", 1, imageCallback); // Publish simultaniously with

    ros::spin();
}
