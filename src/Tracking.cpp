#include "Tracking.h"

using namespace Eigen;

std::vector<std::vector<double>> speed_centers;

// --------- TRACKING Methods --------

VectorXd& Tracking::track_by_most_events(const std::vector<std::vector<double> > &cluster_list, VectorXd &moving_obj){
    double most_events = -1;
    for (const std::vector<double> &aVector : cluster_list){
        bool moreEventsDetected = false;
        int aFeatureIterator = 0;
        for (const double &aFeature : aVector){
            if (aFeatureIterator == 4){
                if (most_events < aFeature){
                    most_events = aFeature;
                    moreEventsDetected = true;
                }
            }
            aFeatureIterator++;
        }
        if (moreEventsDetected){
            moving_obj << aVector[1], aVector[2], most_events; // x, y, #ev
        }
    }
    return moving_obj;
}

MatrixXd& Tracking::object_tracker(const std::vector<std::vector<double> > &kalman_centers){
    // ------------ Calculate speed among all clusters ---------------
    const unsigned int size = kalman_centers.size();
    MatrixXd kalman_centers_mat(size,6);
    VectorXd last_col(size);
    VectorXd col_i(size);
    VectorXd col_j(size);
    std::vector<double> temp_data;
    double x1,y1,x2,y2, speed_of_cluster;

    // INITALIZE WITH ZEROS
    for ( int i = 0; i <= size-1 ; i++){
        for ( int j = 0; j <= 5; j++){ // for all 6 data
            kalman_centers_mat(i,j) = 0;
        }
        col_i(i) = 0;
        col_j(i) = 0;
        last_col(i) = 0;
    }
    
    // Convert to Eigen matrix
    int i = 0;
    for (const std::vector<double> &aVector : kalman_centers){ 
        int j = 0;
        for (const double &aFeature : aVector){
            kalman_centers_mat(i, j) = aFeature;
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
    for ( int i = 0; i <= size-1 ; i++){
        temp_data = {};
        for ( int j = 0; j <= 5; j++){
            temp_data.push_back(kalman_centers_mat(i,j));
        }
        speed_centers.push_back(temp_data);
    }
    sort(speed_centers.begin(),speed_centers.end(), greater<std::vector<double>>());

    object_coordinates << 0, 0, 0, 0;

    // Get the first one(with higher speed) which must be the moving object
    int ObjectIterator = 0;
    for (const std::vector<double> &aVector : speed_centers){
        int aFeatureIterator = 0;
        if (ObjectIterator <= num_of_obj-1){ // Specify the number of moving objects that you want to detect
            for ( const double &aFeature : aVector){
                if ( aFeatureIterator == 0){ speed_of_cluster = aFeature;} // Get the speed of each cluster
                if (speed_of_cluster > speed_limit){
                    if (ObjectIterator==0){
                        if ( aFeatureIterator == 2){x1 = aFeature;}
                        if ( aFeatureIterator == 3){y1 = aFeature;}
                    }
                    if (ObjectIterator==1){
                        if ( aFeatureIterator == 2){x2 = aFeature;}
                        if ( aFeatureIterator == 3){y2 = aFeature;}
                    }
                }
                aFeatureIterator++;
            }
        }
        ObjectIterator++;
    }
    
    object_coordinates << x1, y1,
                            x2, y2;
    return object_coordinates;
}
