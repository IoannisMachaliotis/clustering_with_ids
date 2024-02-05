#include "clustering_with_ids/AEClustering.h"

// ---- Libraries Added ----
#include "stdio.h"
#include <iostream>

AEClustering::AEClustering()
{
    minN_ = 5;
    szBuffer_ = 800;
    szBuffer_ = 800;
    tMin_ = 0;
    radius_ = 25;
    alpha_ = 0.8;
    t0_ = -1;
    kappa_ = 0.5;
    lastUpdatedCluster_ = -1;
    eventId_ = 0;
}

void AEClustering::init(int szBuffer, double radius, double kappa, double alpha, const int minN)
{
    szBuffer_ = szBuffer;
    radius_ = radius;
    alpha_ = alpha;
    minN_ = minN;
    kappa_ = kappa;
}

bool AEClustering::update(const std::deque<double> e)
{
    if (t0_ < 0)
    {
bool AEClustering::update(const std::deque<double> e)
{
    if (t0_ < 0)
    {
        t0_ = e[0];
    }

    Eigen::VectorXd pix(2);
    pix(0) = e[1];
    pix(0) = e[1];
    pix(1) = e[2];

    t = e[0] - t0_;

    std::deque<int> assigned;
    std::deque<int> removed;
    updateBuffer_(t);

    // Evaluate proximity with clusters
    for (int ii = 0; ii < clusters.size(); ii++)
    {

    // Evaluate proximity with clusters
    for (int ii = 0; ii < clusters.size(); ii++)
    {
        clusters[ii].forget(tMin_);

        // Added 2nd condition ---> improves performance by refreshing buffer of events used for a cluster
        if (clusters[ii].getN() == 0 || clusters[ii].getN() > szBuffer_) 
        {
            // Remove the cluster when it is empty or when it exceeds buffer_sz
            removed.push_back(ii);
        }
        else if (clusters[ii].manhattanDistance(pix) <= radius_)
        {
        else if (clusters[ii].manhattanDistance(pix) <= radius_)
        {
            // Assign to the cluster if it is close to the weighted moving average
            assigned.push_back(ii);
        }
        else if (clusters[ii].getN() > minN_)
        {
        else if (clusters[ii].getN() > minN_)
        {
            // Sampling old points from cluster and assign to the cluster if it is close to any of them
            if (clusters[ii].manhattanDistanceWithSampling(pix) <= radius_)
            {
            if (clusters[ii].manhattanDistanceWithSampling(pix) <= radius_)
            {
                assigned.push_back(ii);
            }
        }
    }

    // No proximity -> new cluster
    if (assigned.size() == 0)
    {
    // No proximity -> new cluster
    if (assigned.size() == 0)
    {
        clusters.push_back(MyCluster(alpha_, kappa_));
        clusters[clusters.size() - 1].add(e, eventId_, t0_);
        lastUpdatedCluster_ = clusters.size() - 1;
        clusters[clusters.size() - 1].add(e, eventId_, t0_);
        lastUpdatedCluster_ = clusters.size() - 1;
    }
    else
    {
    else
    {
        lastUpdatedCluster_ = assigned[0];
        clusters[assigned[0]].add(e, eventId_, t0_);

        // Proximity to more than one cluster-> merging
        if (assigned.size() >= 2)
        {
        // Proximity to more than one cluster-> merging
        if (assigned.size() >= 2)
        {
            merge_clusters_(assigned);
            return false;
            return false;
        }
    }

    for (int ii = removed.size() - 1; ii >= 0; ii--)
    {
        if (lastUpdatedCluster_ > removed[ii])
    for (int ii = removed.size() - 1; ii >= 0; ii--)
    {
        if (lastUpdatedCluster_ > removed[ii])
            lastUpdatedCluster_--;

        // Delete the cluster
        // Delete the cluster
        clusters.erase(clusters.begin() + removed[ii]);
    }

    return false;
}

int AEClustering::getLastUpdatedClusterIdx()
{
int AEClustering::getLastUpdatedClusterIdx()
{
    return lastUpdatedCluster_;
}

const int AEClustering::getMinN()
{
    return minN_;
}

/* ************************************************************** *
 *                          PRIVATE                               *
 * ************************************************************** */

void AEClustering::updateBuffer_(double t)
{
void AEClustering::updateBuffer_(double t)
{
    tBuffer_.push_back(t);
    if (tBuffer_.size() > szBuffer_)
    {
    if (tBuffer_.size() > szBuffer_)
    {
        tBuffer_.pop_front();
    }
    tMin_ = tBuffer_[0];
}

void AEClustering::merge_clusters_(std::deque<int> assigned)
{
void AEClustering::merge_clusters_(std::deque<int> assigned)
{

    int m = assigned.size();

    std::deque<int> nn;
    std::deque<std::deque<int>> datId;
    std::deque<std::deque<Eigen::VectorXd>> dat;
    std::deque<std::deque<double>> datT;
    std::deque<std::deque<bool>> datPol;
    std::deque<int> count;

    int aux_n = 0;
    std::deque<int> aux_datId;
    std::deque<Eigen::VectorXd> aux_dat;
    std::deque<double> aux_datT;
    std::deque<bool> aux_datPol;

    for (int ii = 0; ii < m; ii++)
    {
    for (int ii = 0; ii < m; ii++)
    {
        nn.push_back(clusters[assigned[ii]].getN());
        datId.push_back(clusters[assigned[ii]].getDatId());
        dat.push_back(clusters[assigned[ii]].getDat());
        datT.push_back(clusters[assigned[ii]].getDatT());
        datPol.push_back(clusters[assigned[ii]].getDatPol());
        count.push_back(0);
        aux_n += clusters[assigned[ii]].getN();
    }

    Eigen::VectorXd aux_mu(Eigen::VectorXd::Zero(2));
    for (int ii = 0; ii < m; ii++)
    {
        aux_mu += ((double)clusters[assigned[ii]].getN() / (double)aux_n) * clusters[assigned[ii]].getMu();
    for (int ii = 0; ii < m; ii++)
    {
        aux_mu += ((double)clusters[assigned[ii]].getN() / (double)aux_n) * clusters[assigned[ii]].getMu();
    }


    int idx = 1;
    double tt;
    while (idx >= 0)
    {
    while (idx >= 0)
    {
        idx = -1;
        tt = std::numeric_limits<double>::max();
        for (int jj = 0; jj < m; jj++)
        {
            if (count[jj] < nn[jj])
            {
                if (datT[jj][count[jj]] < tt)
                {
        for (int jj = 0; jj < m; jj++)
        {
            if (count[jj] < nn[jj])
            {
                if (datT[jj][count[jj]] < tt)
                {
                    idx = jj;
                    tt = datT[jj][count[jj]];
                }
            }
        }
        if (idx >= 0)
        {
        if (idx >= 0)
        {
            aux_datId.push_back(datId[idx][count[idx]]);
            aux_dat.push_back(dat[idx][count[idx]]);
            aux_datT.push_back(datT[idx][count[idx]]);
            aux_datPol.push_back(datPol[idx][count[idx]]);
            count[idx]++;
        }
    }

    clusters[assigned[0]].setN(aux_dat.size());
    clusters[assigned[0]].setDatId(aux_datId);
    clusters[assigned[0]].setDat(aux_dat);
    clusters[assigned[0]].setDatT(aux_datT);
    clusters[assigned[0]].setDatPol(aux_datPol);
    clusters[assigned[0]].setMu(aux_mu);

    for (int ii = m - 1; ii > 0; ii--)
    {
        clusters.erase(clusters.begin() + assigned[ii]);
    }
}
    for (int ii = m - 1; ii > 0; ii--)
    {
        clusters.erase(clusters.begin() + assigned[ii]);
    }
}
