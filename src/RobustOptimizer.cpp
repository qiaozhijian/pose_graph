//
// Created by zmy on 2021/2/5.
//

#include "RobustOptimizer.h"
#include <iostream>
#include "glog/logging.h"
//#include "Thirdparty/g2o/g2o/stuff/sampler.h"
//#include "Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
using namespace std;

RobustOptimizer::RobustOptimizer(const bool& verbose, const int& maxIteration):Optimizer(verbose,maxIteration) {

    LOG(INFO) << "RobustOptimizer created." << endl;

}


void RobustOptimizer::runOptimizer(int level) {

    if(!graph->initializeOptimization(level)){
        LOG(ERROR) << "First Graph initialize Optimization Wrong!!!";
        return ;
    }

    for(auto e:unary_edges) {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        rk->setDelta(1);
        e->setRobustKernel(rk);
    }

    // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
    double chi2 = graph->activeChi2();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    int iterations = graph->optimize(_MaxIterationTimes);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tcost= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    LOG(INFO) << "First optimize: chi2: (before)" << chi2 << " -> (after) " <<  graph->activeChi2() << ", iterations: " << iterations
              << ", time: " << tcost << "[sec]" << std::endl;


    RemoveOutliers();

    if(!graph->initializeOptimization(level)){
        LOG(ERROR) << "Second Graph initialize Optimization Wrong!!!";
        return ;
    }

    chi2 = graph->activeChi2();
    t1 = std::chrono::steady_clock::now();
    iterations = graph->optimize(_MaxIterationTimes);
    t2 = std::chrono::steady_clock::now();
    tcost= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    LOG(INFO) << "Second optimize: chi2: (before)" << chi2 << " -> (after) " <<  graph->activeChi2() << ", iterations: " << iterations
              << ", time: " << tcost << "[sec]" << std::endl;


}

void RobustOptimizer::RemoveOutliers(){
    int n = 0;
    for(auto e:unary_edges) {
        n++;
        if (e->chi2() > 3) {
            e->setLevel(1);
            LOG(INFO) << "edge" << n << "  is an outlier, which chi2 is " << e->chi2() << endl;
        }
        e->setRobustKernel(0);
    }
    return ;
}

