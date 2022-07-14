#include "Improve.h"

#define CLUSTERS 15

// ----- Visualization settings -----
bool accuracy_visual = false;

using namespace Eigen;
using namespace std;


// ------ For Accuracy -------
MatrixXd Extracted_cen(CLUSTERS, 2);   // Extracted Centers
MatrixXd Predicted_cen(CLUSTERS, 2);   // Predicted Centers
VectorXd Accuracy_mat(CLUSTERS, 1);    // Accuracy
MatrixXd Velocity_mat(CLUSTERS, 2);    // Velocity

double dt;
// -------------- KALMAN FILTER VARIABLES ----------------
int n = 3;                             // Number of states (position, velocity, acceleration)
int m = 1;                             // Number of measurements


MatrixXd A(n, n);                      // State Matrix
MatrixXd C(m, n);                      // Output matrix
MatrixXd Q(n, n);                      // Process noise covariance (keeps the state covariance matrix from becoming too small, or going to 0)
MatrixXd R(m, m);                      // Measurement covariance matrix (Error in the measurement)
MatrixXd P(n, n);                      // State covariance matrix (Error in the estimate)
MatrixXd I(n, n);                      // Identity Matrix
MatrixXd P_previous(n, n);
MatrixXd K;                            // Kalman Gain (based on comparing the error in the estimate to the error in the measurement)
VectorXd x_hat(m, n), x_hat_new(m, n); // Estimated states
VectorXd x_hat_previous(m, n);
VectorXd y(m);
VectorXd x0(n);

VectorXd v_temp(3);

// ------ For last Kalman Centers (Feedback) ------
MatrixXd previous_KF_centers(CLUSTERS, 2);


VectorXd Eucleidian_acc(vector<vector<double>> &cluster_centers, vector<vector<double>> &kalman_centers){
    int size = cluster_centers.size();

    // Initialize Matrices
    for (int i = 0; i < CLUSTERS; i++){
        for (int j = 0; j <= 1; j++){
            Extracted_cen(i, j) = 0;
            Predicted_cen(i, j) = 0;
            Velocity_mat(i, j) = 0;
        }
        Accuracy_mat(i) = 0;
    }

    // Convert from std to Eigen vectors for processing
    int counter = 0;
    for (vector<double> v : cluster_centers){
        int j = 0;
        for (double A : v){
            if (j != 0){
                Extracted_cen(counter, j - 1) = A;
            }
            j++;
        }
        counter++;
    }

    int counter1 = 0;
    for (vector<double> v : kalman_centers){
        int j = 0;
        for (double A : v){
            if (j != 0){
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
    for (int i = 0; i < size; i++){
        for (int j = 0; j < 1; j++){
            if (j == 0){
                x_ex = Extracted_cen(i, j);
                x_pr = Predicted_cen(i, j);
            }
            if (j == 1){
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


// --------------- KALMAN FILTER ----------------

VectorXd KF_algorithm(VectorXd &y){
    x_hat_new = A * x_hat;
    P = A * P * (A.transpose()) + Q;
    K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse();
    P = (I - K * C) * P * (I - K * C).transpose() + K * R * K.transpose();
    x_hat_new += K * (y - C * x_hat_new);

    return x_hat_new;
}


// 1st member
vector<vector<double> >& Improve::remover(vector<vector<double> >& cluster_centers){
    double buff_limit = 0.01; // seconds (how fast to remove)
    double Index;
    double duration = 0;

    int counter = 0;
    for (vector<double> v : cluster_centers){
        int k = 0;
        for (double i : v){
            if (k == 3){
                ros::Time checkpoint = ros::Time::now();
                duration = checkpoint.toSec() -  i;
            }
            k++;
        }
        if (duration > buff_limit){
            Index = counter;
            cluster_centers.erase(cluster_centers.begin() + Index);
        }
        counter++;
    }
    return cluster_centers;
}

// 2nd member
vector<vector<double> >& Improve::kalmanfilter(vector<vector<double> >& cluster_centers, vector<vector<double> >& kalman_centers){
    dt = 20;

    // ------------- 2D IMPLEMENTATION of State Space Kalman Filter ---------------
    vector<vector<double>> position_vector = {};
    vector<double> temporary_IDs = {};
    vector<double> temporary_num_of_ev = {};
    vector<double> temporary_t_stamp = {};
    vector<double> temporary_speed = {};
    // ------- OBTAIN info FROM cluster_centers(to assign back afterwards) --------
    int counter1 = 0;
    for (vector<double> v : cluster_centers){
        vector<double> temp_xy = {};
        int k = 0;
        for (double i : v){
            if (k == 0){                             // IDs
                temporary_IDs.push_back(i);
            }
            if (k == 1 || k == 2){                   // 2D(x&y_axis_position)
                temp_xy.push_back(i);
            }
            if (k == 3){                             // t_stamp
                temporary_t_stamp.push_back(i);
            }
            if (k == 4){                             //# events
                temporary_num_of_ev.push_back(i);
            }
            if (k == 5){                             // speed
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
    for (double i : temporary_IDs){                     // IDs
        temp_IDs(counter_1) = i;
        counter_1++;
    }
    int counter_2 = 0;
    for (vector<double> v : position_vector){           // Position
        int j = 0;
        for (double A : v){
            POS_MEAS_MAT_2D(counter_2, j) = A;
            j++;
        }
        counter_2++;
    }
    int counter_3 = 0;
    for (double i : temporary_t_stamp){                  // t_stamps
        temp_t_stamp(counter_3) = i;
        counter_3++;
    }
    int counter_4 = 0;
    for (double i : temporary_num_of_ev){                // # of events
        temp_num_ev(counter_4) = i;
        counter_4++;
    }
    int counter_5 = 0;
    for (double i : temporary_speed){                    // speed
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
        0.5 * pow(dt, 2), 0.5 * pow(dt, 2), .0,                    // Mean of measurements covariance
        .0, .0, .0;

    R << 1;           // Observation Covariance
    P << .1, .1, .1,
        .1, 10000, 1, // Current State covariance
        .1, 1, 100;

    // ----- ESTIMATE KALMAN CENTERS based on the info obtained ------
    if (POS_MEAS_MAT_2D.isZero() == 0){                             // If there is any cluster in Eigen Matrix
        for (int i = 0; i < POS_MEAS_MAT_2D.rows(); i++){
            vector<double> temp_updated_center = {};                // Create std::vector which will be pushed_back every 2 iterations
            for (int j = 0; j < POS_MEAS_MAT_2D.cols(); j++){
                y << POS_MEAS_MAT_2D(i, j);                         // current measurement

                if (previous_KF_centers.row(i).isZero() == 1){
                    x0 << y, 0, 0;                                  // Best guess of initial states
                }
                else{
                    x0 << previous_KF_centers(i, j), 0, 0;
                }
                x_hat = x0;                                         // INITIALIZE state

                v_temp << KF_algorithm(y);                          // UPDATE MEASUREMENT

                double ii = v_temp(0);                              // Create kalman_centers list

                Velocity_mat(i, j) = v_temp(1);                     // Assign Velocity

                // cout << Velocity_mat << "\n";

                if (j == 0){
                    // Assign the ID back
                    temp_updated_center.push_back(temp_IDs(i)); // vector --> {ID}
                }
                temp_updated_center.push_back(ii);              // vector --> {ID, x, y}
                if (j == 1){
                    // Assign the timestamps back                  vector --> {ID, x, y, t_stamp}
                    temp_updated_center.push_back(temp_t_stamp(i));

                    // Assign the # of events back                 vector --> {ID, x, y, t_stamp, #ev}
                    temp_updated_center.push_back(temp_num_ev(i));

                    // Assign the speed back                       vector --> {ID, x, y, t_stamp, #ev, speed}
                    temp_updated_center.push_back(temp_speed(i));
                }

                if (temp_updated_center.size() == 6){            // Every 2 iterations (x & y)
                    kalman_centers.push_back(temp_updated_center);
                    temp_updated_center.erase(temp_updated_center.begin(), temp_updated_center.end());
                }
            }
        }
    }

    // Delete rows that do not exist in new kalman_centers
    int previous_KF_size = 0;
    for (int i = 0; i < previous_KF_centers.rows(); i++){
        if (previous_KF_centers(i, 0) != 0){
            previous_KF_size++;
        }
    }
    if (previous_KF_size > kalman_centers.size()){
        for (int i = kalman_centers.size(); i < previous_KF_centers.rows(); i++){
            for (int j = 0; j <= 1; j++){
                previous_KF_centers(i, j) = 0;
            }
        }
    }

    // CONVERT kalman_centers to EIGEN MATRICES and store for next iteration
    int counter3 = 0;
    for (vector<double> v : kalman_centers){
        int j = 0;
        for (double A : v){
            if (j != 0){
                previous_KF_centers(counter3, j - 1) = A;
            }
            j++;
        }
        counter3++;
    }

    // Accuracy measurement
    Accuracy_mat << Eucleidian_acc(cluster_centers, kalman_centers);

    // Choose if you want accuracy visualized
    if (accuracy_visual){
        int counter6 = 0;
        for (vector<double> v : kalman_centers){
            int num = (1 - Accuracy_mat(counter6)) * 100;
            string s = "Impr:" + to_string(num) + "%";

            cv::putText(im2, s, cv::Point(v[1] + 4, v[2] - 14), cv::FONT_ITALIC, 0.3, cv::Scalar(50, 50, 255), 0, 16);

            counter6++;
        }
        cout << "Accuracy for each cluster is:\n" << Accuracy_mat << "\n";
    }
    return kalman_centers;
}