/* Code that prints all the visualized clusters from opencv (green clusters)
   to the terminal 

   The goal is to save also previous moment clusters
*/

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
#include <vector>
#include <math.h>

AEClustering *eclustering(new AEClustering);


image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;

std::vector<std::vector<int>> cluster_centers; // Prosoxi ta vectors einai 2 diastasewn!!
std::vector<std::vector<int>> previous_clusters;
int new_count;
int previous_count;
int dif;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        int Id=1;// Create Cluster ID
        int minN = eclustering->getMinN();// 15
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image;
        std::cout << "NEW CLUSTERS..\n";

/*               
                // SAVE PREVIOUS CLUSTERS
                for (std::vector<int> n : cluster_centers) {
                    previous_clusters.push_back(n);
                }
*/
        for (auto cc : eclustering->clusters){

            if (cc.getN() >= minN){//αν ο αριθμος των δεδομενων που συνεβαλαν για να δημιουργηθει ο cluster ειναι μεγαλυτερος του 15

                // Erase clusters for update
                cluster_centers.erase(cluster_centers.begin(), cluster_centers.end());

                Eigen::VectorXd cen(cc.getClusterCentroid());

                //Convert cen vector from eigen vector to std::vector
                std::vector<int> cen_converted(&cen[0], cen.data()+cen.cols()*cen.rows());

                if (Id<80){ //choose which Ids to see
                    cv::circle(im,cv::Point(cen[0],cen[1]), 2, cv::Scalar(0,255,0,255), -1);

                    //save new cluster center, generated with ID, in a VECTOR
                    cluster_centers.push_back(cen_converted);

                    //SHOW NEW CLUSTERS
                    //show ID and center of new clusters
                    std::cout << "ID:"<< Id << "   Center: ";
                    for (std::vector<int> n : cluster_centers){
                        int j=0;
                        int k=0;
                        for ( int i : n) {
                            std::cout << i;
                            if (k%2==0){std::cout << ",";}
                            k++;
                        }
                        j++;
                        if (j%2==0) {
                            std::cout << " \n";
                        }
                    }
                    std::cout << "\n";

                    Id++;
                }
            }
        }
        /*
                    //SHOW PREVIOUS CLUSTERS
//                    std::cout << "                           PREVIOUS CLUSTERS..\n";
                    int m=1;
                    for (std::vector<int> n : previous_clusters){
                        int j=0;

                        //show ID and center of previous clusters
                        std::cout << "                          ID:"<< m << "   Center: ";
                        int k=0;
                        for ( int i : n) {
                            std::cout << i;
                            if (k%2==0){std::cout << ",";}
                            k++;
                        }
                        j++;
                        if (j%2==0) {
                            std::cout << "\n";
                        }
                        m++;
                    }
                    std::cout << "\n";

                    // Erase previous clusters for update
                    previous_clusters.erase(previous_clusters.begin(), previous_clusters.end());
*/
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
/*    
    std::cout << new_count << " new clusters\n";

    previous_count = new_count; 
    std::cout << previous_count << " previous clusters\n"; 

    //ERASE PREVIOUS CLUSTERS FOR UPDATE
    previous_clusters.erase(previous_clusters.begin(), previous_clusters.end());
*/
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


    eclustering->init(szBuffer, radius, kappa, alpha, minN);

    ros::Subscriber subEv = nh.subscribe("/dvs/events", 1, eventCallback);

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("clusters_image", 1);
    image_transport::Subscriber subIm = it.subscribe("dvs/image_raw", 1, imageCallback);


    ros::spin();

}
