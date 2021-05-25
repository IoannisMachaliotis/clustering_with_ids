#include "MyCluster.h"

#ifndef AECLUSTERING_CLASS_H
#define AECLUSTERING_CLASS_H

class AEClustering{
    private:

        int minN_;
        int szBuffer_;
        std::deque<double> tBuffer_;
        double tMin_; 
        double radius_;
        double alpha_;
        int kappa_;
        int eventId_;
        double t0_;
        int lastUpdatedCluster_;

        void updateBuffer_(double t);
        
        void merge_clusters_(std::deque<int> assigned);
    
    public:

        double t;
        std::deque<MyCluster> clusters;

        AEClustering();

        void init(int szBuffer, double radius=25, int kappa=0, double alpha=0.5, int minN=5);

        bool update(const std::deque<double> e);
    
        int getLastUpdatedClusterIdx();

        int getMinN();

};

#endif

