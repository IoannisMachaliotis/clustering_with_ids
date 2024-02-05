#include "Tracking.h"

using namespace Eigen;
using namespace std;

vector<vector<double>> speed_centers;

// --------- TRACKING Methods --------

VectorXd& Tracking::track_by_most_events(const vector<vector<double> >& n, VectorXd& moving_obj){
    double most_events = -1;
    for (vector<double> v : n){
        bool flag = false;
        int k = 0;
        for (double i : v){
            if (k == 4){
                if (most_events < i){
                    most_events = i;
                    flag = true;
                }
            }
            k++;
        }
        if (flag){
            moving_obj << v[1], v[2], most_events; // x, y, #ev
        }
    }
    return moving_obj;
}

MatrixXd& Tracking::object_tracker(vector<vector<double> >& kalman_centers){
    // ------------ Calculate speed among all clusters ---------------
    int size = kalman_centers.size();
    MatrixXd kalman_centers_mat(size,6);
    VectorXd last_col(size);
    VectorXd col_i(size);
    VectorXd col_j(size);
    vector<double> temp_data;
    double x1,y1,x2,y2, speed_of_cluster;

    // INITALIZE WITH ZEROS
    for (int i = 0; i <= size-1 ; i++){
        for ( int j = 0; j <= 5; j++){ // for all 6 data
            kalman_centers_mat(i,j) = 0;
        }
        col_i(i) = 0;
        col_j(i) = 0;
        last_col(i) = 0;
    }
    
    // Convert to Eigen matrix
    int i = 0;
    for (vector<double> v : kalman_centers){ 
        int j = 0;
        for (double A : v){
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
    for (int i = 0; i <= size-1 ; i++){
        temp_data = {};
        for ( int j = 0; j <= 5; j++){
            temp_data.push_back(kalman_centers_mat(i,j));
        }
        speed_centers.push_back(temp_data);
    }
    sort(speed_centers.begin(),speed_centers.end(), greater<vector<double>>());

    object_coordinates << 0, 0, 0, 0;

    // Get the first one(with higher speed) which must be the moving object
    int kk = 0;
    for ( vector<double> v : speed_centers){
        int k = 0;
        if (kk <= num_of_obj-1){ // Specify the number of moving objects that you want to detect
            for ( double p : v){
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
    return object_coordinates;
}
