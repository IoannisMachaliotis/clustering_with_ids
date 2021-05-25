#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <deque>
#include <limits>

#ifndef MYCLUSTER_CLASS_H
#define MYCLUSTER_CLASS_H

class MyCluster{
    private:

        std::deque<int> datId_;
        std::deque<Eigen::VectorXd> dat_;
        std::deque<double> datT_;
        std::deque<bool> datPol_;
        double alpha_;
        Eigen::VectorXd mu_;
        int n_;
        int kappa_;

        void updateMu_(Eigen::VectorXd pix);
        
    public:

        MyCluster();

        MyCluster(double alpha, int k);

        void reset(int kappa, double alpha, int minN);

        void add(const std::deque<double> e, int &eventId, double t0);

        void forget(double t);

        double manhattanDistance(Eigen::VectorXd x);

        double manhattanDistanceWithSampling(Eigen::VectorXd x);


        void setN(int n);

        void setDatId(std::deque<int> datId);

        void setDat(std::deque<Eigen::VectorXd> dat);

        void setDatT(std::deque<double> datT);

        void setDatPol(std::deque<bool> datPol);

        void setMu(Eigen::VectorXd mu);

        int getN();

        std::deque<int> getDatId();

        std::deque<Eigen::VectorXd> getDat();

        std::deque<double> getDatT();  

        std::deque<bool> getDatPol();  

        Eigen::VectorXd getClusterCentroid();

        Eigen::VectorXd getMu();


};

#endif
