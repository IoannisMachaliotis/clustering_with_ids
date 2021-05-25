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



void imageCallback(const sensor_msgs::ImageConstPtr& msg){

/*        double duration,start;
	int flag;
	int argc; 
	char** argv;
*/
    try{
	int Id=1;// Create Cluster ID

	int cluster_centers[20][2];
//        int previous_centers[2];
        int minN = eclustering->getMinN();// 15
//	std::cout << minN<< "\n";
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image; 
        for (auto cc : eclustering->clusters){
            if (cc.getN() >= minN){//αν ο αριθμος των δεδομενων που συνεβαλαν για να δημιουργηθει ο cluster ειναι μεγαλυτερος του 15
                Eigen::VectorXd cen(cc.getClusterCentroid());
               // cv::circle(im,cv::Point(cen[0],cen[1]), 2, cv::Scalar(0,255,0,255), -1);
		if (Id<50){
		//save new cluster center, generated with ID, in a list
                for (int j=0; j<=1; j++){
                      cluster_centers[Id][j]=cen[j];
			if (Id<=25){
			cv::circle(im,cv::Point(cen[0],cen[1]), 2, cv::Scalar(0,255,0,255), -1);
			}
                }

		//show ID and center
                std::cout << "ID:"<< Id << "   Center:";
		for (int j=0; j<=1; j++){
                       	std::cout << cluster_centers[Id][j];
               	        if (j==0) {std::cout << ",";}
                       	std::cout << " ";
                }
		std::cout << "\n";
		}
		//refresh list of centers
                if (Id>15){
                        Id=1;
                }
		Id++; 
	     }
	}
/*
	//stop the clock
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout <<"Duration is "<< duration << "sec \n";
	if (duration>5){ 
		flag=1;

	duration = callback()-start;
	std::cout << duration << " sec\n";
*/

        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
//    return flag;
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

/*LOOP:
	 //start the clock
        std::clock_t start;
        start = std::clock();
*/    
    ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh("~");

//    int flag=0;
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

/*
	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
 	spinner.spin();
*/
/*
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
*/
	ros::spin();
//    ros::Rate loop_rate(100);
/*	while (1){
	    ros::spinOnce();
	}
*/
/*
    //refresh node
        if (std::clock()-start>10){
//                ros::shutdown();
		ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
		ros::NodeHandle nh_public;
		ros::NodeHandle nh("~");
                goto LOOP;
        }else{
        goto SPIN;
        }
*/
}

