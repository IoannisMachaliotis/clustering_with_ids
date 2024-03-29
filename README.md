# clustering_with_ids (ID speed Tracker) 
This repository includes the ROS implemenatation of the event-based clustering algorithm described in the paper "**Asynchronous Event-Based Clustering and Tracking for Intrusion Monitoring in UAS**".


# Requirements
* [Eigen 3](https://eigen.tuxfamily.org/dox/)
* [OpenCV 4.2](https://opencv.org/opencv-4-2-0/)
* [ROS Noetic](http://wiki.ros.org/kinetic) -> with Linux Ubuntu (confirmed up until 20.04) _WSL incompatible for now_
* [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) 


# Installation
Clone the repository to your ROS workspace (e.g ~/catkin_ws/src) 


    $ sudo git clone https://github.com/IoannisMachaliotis/clustering_with_ids.git
    
**VERY IMPORTANT** to source the new setup.bash instead of the initial ~/.bashrc **AND** to build all packages only from the ~/catkin_ws directory
(in **every** terminal: roscore, bagfile play, renderer, id_speed_tracker)  

    $ cd ..
    $ catkin build clustering_with_ids
    $ source /devel/setup.bash 

The package includes a wrapper as example to use the tracker algorithm. The wrapper receives the input using the `dvs_msgs/EventArray` message format. To try the clustering, connect your camera and run the driver. Then type:

    $ roslaunch clustering_with_ids test_tracker_with_ids.launch

# License
This code is released under MIT license

I constantly work to improve algorithm performance and code reliability. For additional information please contact me at <johnmax1996@gmail.com>

# Preview

Take a look at the simulation preview at my channel on [Youtube](https://www.youtube.com/channel/UCNrjasEwN54DjObTZaLTm0w/playlists)
