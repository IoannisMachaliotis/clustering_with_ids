#include <ros/callback_queue.h>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "grvc_e_clustering/AEClustering.h"


AEClustering *eclustering(new AEClustering);


image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;
//next step is to make the above arrays dynamic so that i can delete line 113
int cluster_centers[70][2];
int previous_clusters[70][2];
int new_count;
int previous_count;
int dif;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        int Id=1;// Create Cluster ID

        //Initialize clusters array so that I can count the elements after
        for (int i=0; i<sizeof(cluster_centers)/sizeof(cluster_centers[0]); i++){
                for (int j=0; j<=1;j++ ){
                        cluster_centers[i][j]=0;
                }
        }

        int minN = eclustering->getMinN();// 15
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
        std::cout << "NEW CLUSTERS..\n";
        for (auto cc : eclustering->clusters){
            if (cc.getN() >= minN){//αν ο αριθμος των δεδομενων που συνεβαλαν για να δημιουργηθει ο cluster ειναι μεγαλυτερος του 15
                Eigen::VectorXd cen(cc.getClusterCentroid());
//              cv::circle(im,cv::Point(cen[0],cen[1]), 2, cv::Scalar(0,255,0,255), -1);

              if (Id<80){ //choose which Ids to see

                //save new cluster center, generated with ID, in a list
                for (int j=0; j<=1; j++){
                      cluster_centers[Id][j]=cen[j];
                        cv::circle(im,cv::Point(cen[0],cen[1]), 2, cv::Scalar(0,255,0,255), -1);
                }

                //show ID and center of new clusters
                std::cout << "ID:"<< Id << "   Center:";
                for (int j=0; j<=1; j++){
                        std::cout << cluster_centers[Id][j];
                        if (j==0) {std::cout << ",";}
                        std::cout << " ";
                }
                std::cout << "\n";
              }
                Id++;
                new_count=0;
            }
        }
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }


    // Count new clusters
    for (int i=0; i<sizeof(cluster_centers)/sizeof(cluster_centers[0]); i++){
            for (int j=0; j<=1; j++){
                    if (cluster_centers[i][j]!=0){
//                          previous_clusters[i][j]=cluster_centers[i][j];
                            new_count++;
                    }
            }
    }
    new_count/=2;

   // Save previous clusters
   for (int i=1; i<=previous_count; i++){
        for (int j=0; j<=1; j++){
                previous_clusters[i][j]=cluster_centers[i][j];
        }
   }

        //Update command window
//    std::cout << "-----------------<<<<>>>>-------------------\n";
    std::cout << "New number of clusters is " << new_count << "\n";
    std::cout << "Previous number of clusters was "<< previous_count << "\n";

    // Compare previous with new
    if (new_count>previous_count){
           dif=new_count-previous_count;
           std::cout << dif <<" new clusters created\n";
    }
    if (new_count<previous_count){
           dif=previous_count-new_count;
           std::cout << dif <<" clusters merged or deleted\n";
    }

    // Show previous clusters
    std::cout << "                              PREVIOUS CLUSTERS..\n";
    for (int i=1; i<=previous_count; i++){
        std::cout << "                          ID:"<< i << "   Center:";
         for (int j=0; j<=1; j++){
                if (previous_clusters[i][j]!=0) {
                        std::cout << previous_clusters[i][j];
                        if (j==0) {std::cout << ",";}
                        std::cout << " ";
                }
          }
          std::cout << "\n";
    }

    previous_count=new_count;
    std::cout << "-----------------<<<<>>>>-------------------\n";

}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg){
    std::deque<double> ev(4, 0.0);
    for (const auto e : msg->events){
        ev[0] = e.ts.toSec();       
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity; 
        eclustering->update(ev);
    }

}

int main(int argc, char** argv)
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
}
