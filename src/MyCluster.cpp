#include "clustering_with_ids/MyCluster.h"

MyCluster::MyCluster(){
    mu_ = Eigen::VectorXd::Zero(2);
    n_ = 0;
    alpha_= 0.8; // percentage of previous info storing??
    kappa_ = 0.5;
}

MyCluster::MyCluster(double alpha, double kappa){
    mu_ = Eigen::VectorXd::Zero(2);
    n_ = 0;
    alpha_ = alpha;
    kappa_ = kappa;
}

void MyCluster::reset(double kappa, double alpha, int minN){ // Not used anywhere?
    kappa_ = kappa;
    alpha_ = alpha;
//    minN = 5; //???
}

void MyCluster::add(const std::deque<double> e, int &eventId, double t0){
    double t = e[0] - t0;

    Eigen::VectorXd pix(2);
    pix(0) = e[1];
    pix(1) = e[2];

    bool pol = e[3];

    datId_.push_back(eventId);
    dat_.push_back(pix);
    datT_.push_back(t);
    datPol_.push_back(pol);

    if(n_ == 0){
        mu_[0] = pix[0];
        mu_[1] = pix[1];
    }
    else{
        updateMu_(pix);       
    }
    n_++;
    eventId++;

}

void MyCluster::forget(double t){
    while(datT_[0] < t && n_ > 0){
        Eigen::VectorXd pixx(dat_[0]);
        bool pol = datPol_[0];
        dat_.pop_front();
        datId_.pop_front();
        datT_.pop_front();
        if(datPol_.size() > 0)
            datPol_.pop_front();
        n_--;
    }
}

double MyCluster::manhattanDistance(Eigen::VectorXd x){
    double ma = abs(x[0] - mu_[0]) + abs(x[1] - mu_[1]);
    return ma;
}

double MyCluster::manhattanDistanceWithSampling(Eigen::VectorXd x){

    int idx = 0;
    double ma = std::numeric_limits<double>::max();
    double foo = 0.0;

    if(kappa_ > n_){
        for (const auto y : dat_){
            foo = abs(x[0] - y[0]) + abs(x[1] - y[1]);
            if(foo < ma){
                ma = foo;
            }
        }
    }
    else{
        for(int ii = 0; ii < kappa_; ++ii){
            idx = std::rand();
            idx = idx % dat_.size();
            Eigen::VectorXd y(dat_[idx]);

            foo = abs(x[0] - y[0]) + abs(x[1] - y[1]);
            if(foo < ma){
                ma = foo;
            }
        }
    }
    return ma;
}

 /* ************************************************************** *
 *                          SETTERS                               *
 * ************************************************************** */

void MyCluster::setN(int n){
    n_ = n;
}

void MyCluster::setDatId(std::deque<int> datId){
    datId_ = datId;
}

void MyCluster::setDat(std::deque<Eigen::VectorXd> dat){
    dat_ = dat;
}

void MyCluster::setDatT(std::deque<double> datT){
    datT_= datT;
}

void MyCluster::setDatPol(std::deque<bool> datPol){
    datPol_= datPol;
}

void MyCluster::setMu(Eigen::VectorXd mu){
    mu_ = mu;
}

/* ************************************************************** *
 *                          GETTERS                               *
 * ************************************************************** */

int MyCluster::getN(){
    return n_;
}

std::deque<int> MyCluster::getDatId(){
    return datId_;
}

std::deque<Eigen::VectorXd> MyCluster::getDat(){
    return dat_;
}

std::deque<double> MyCluster::getDatT(){
    return datT_;
}

std::deque<bool> MyCluster::getDatPol(){
    return datPol_;
}

Eigen::VectorXd MyCluster::getMu(){
    return mu_;
}

Eigen::VectorXd MyCluster::getClusterCentroid(){
    double xAcc = 0;
    double yAcc = 0;
    for (int ii = 0; ii < dat_.size(); ++ii){
        xAcc = xAcc + dat_[ii](0);
        yAcc = yAcc + dat_[ii](1);
    }

    xAcc = xAcc / dat_.size();
    yAcc = yAcc / dat_.size();
    
    Eigen::VectorXd centroidDat(2);
    centroidDat[0] = xAcc; 
    centroidDat[1] = yAcc;
    return centroidDat;
}

/* ************************************************************** *
 *                          PRIVATE                               *
 * ************************************************************** */

void MyCluster::updateMu_(Eigen::VectorXd pix){
    mu_ = (1-alpha_) * mu_ + alpha_ * pix;
//    mu_ = pix;
}
