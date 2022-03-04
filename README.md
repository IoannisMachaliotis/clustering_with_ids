# clustering_with_ids 
This repository includes the ROS implemenatation of the event-based clustering algorithm described in the paper "**Asynchronous Event-Based Clustering and Tracking for Intrusion Monitoring in UAS**".


# Requirements
* [Eigen 3](https://eigen.tuxfamily.org/dox/)
* [OpenCV 3.3](https://opencv.org/opencv-3-3/)
* [ROS Kinetic](http://wiki.ros.org/kinetic) 
* [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) 


# Installation
Clone the repository to your ROS workspace (e.g ~/catkin_ws/src) 


    $ sudo git clone https://github.com/IoannisMachaliotis/clustering_with_ids.git
    $ catkin build clustering_with_ids
    $ source ~/catkin_ws/devel/setup.bash

The package includes a wrapper as example to use the tracker algorithm. The wrapper receives the input using the `dvs_msgs/EventArray` message format. To try the clustering, connect your camera and run the driver. Then type:

    $ roslaunch clustering_with_ids test_tracker_with_ids.launch

# License
This code is released under MIT license

We constantly work to improve algorithm performance and code reliability. For additional information please contact Ioannis Machaliotis <johnmax1996@gmail.com>
