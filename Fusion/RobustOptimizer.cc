//
// Created by zmy on 2021/2/5.
//

#include "RobustOptimizer.h"
#include <iostream>
#include "glog/logging.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
using namespace std;

vector<g2o::EdgeSE3Prior*> Myedges;

RobustOptimizer::RobustOptimizer(const bool& Verbose, const int& maxIteration) {

    graph = new g2o::SparseOptimizer();
    _Verbose = Verbose;
    _MaxIterationTimes = maxIteration;


    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 稠密线性求解器
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // 设置求解器
    graph->setAlgorithm(solver);
    //打开调试输出
    graph->setVerbose(_Verbose);


    if (!graph->solver()){
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
    }

    //有些程序加了这三句话，但没有给出原因
    g2o::ParameterSE3Offset* se3Offset = new g2o::ParameterSE3Offset;
    se3Offset->setId(0);
    graph->addParameter(se3Offset);

    LOG(INFO) << "Optimizer created." << endl;

}

g2o::VertexSE3* RobustOptimizer::addSE3Node(const Eigen::Isometry3d &pose, const int KFid, const bool& isFixed) {
    auto vertex(new g2o::VertexSE3());
    g2o::SE3Quat pose_ = g2o::internal::toSE3Quat(pose);
    vertex->setId(KFid);
    vertex->setEstimate(pose_);
    vertex->setFixed(isFixed);
    graph->addVertex(vertex);

    return vertex;
}

void RobustOptimizer::addSE3Edge(g2o::VertexSE3* &v1, g2o::VertexSE3* &v2,
                                 const Eigen::Isometry3d &relative_pose,
                                 const Matrix6d &info_matrix, int level) {

    auto edge = new g2o::EdgeSE3();
    edge->setMeasurement(relative_pose);
    edge->setInformation(info_matrix);
    edge->setVertex(0, v1);
    edge->setVertex(1, v2);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    edge->setRobustKernel(rk);
    rk->setDelta(1);
    edge->setLevel(level);
    graph->addEdge(edge);

}

void RobustOptimizer::addSE3Edge(g2o::VertexSE3* &v1, const Eigen::Isometry3d &iso_pose,
                                 const Matrix6d &info_matrix, int level) {

    auto edge = new g2o::EdgeSE3Prior();
    edge->setVertex(0, v1);
    edge->setMeasurement(iso_pose);
    edge->setInformation(info_matrix);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    edge->setRobustKernel(rk);
    rk->setDelta(1);
    edge->setParameterId(0, 0);
    edge->setLevel(level);
    graph->addEdge(edge);
    Myedges.push_back(edge);
    return;
}

void RobustOptimizer::addSE3PlaneEdge(g2o::VertexSE3* &v1, int level) {

    Matrix6d info = Matrix6d::Zero(6, 6);
//    z, roll ,pitch 为 0
    info(2, 2) = 1e9;
    info(3, 3) = 1e9;
    info(4, 4) = 1e9;

    auto edge = new g2o::EdgeSE3Prior();
    edge->setVertex(0, v1);
    edge->setMeasurement(Eigen::Isometry3d::Identity());
    edge->setInformation(info);
    edge->setParameterId(0, 0);
    edge->setLevel(level);
    graph->addEdge(edge);

}

void RobustOptimizer::runOptimizer(int level) {

    LOG(INFO) << std::endl
              << "--- pose graph optimization ---" << std::endl
              << "nodes: " << graph->vertices().size() << "   "
              << "edges: " << graph->edges().size() << std::endl
              << "optimizing... ";
    LOG(INFO) << "Chi2: " << graph->activeChi2() << endl;
    if(!graph->initializeOptimization(level)){
        LOG(ERROR) << "Graph initialize Optimization Wrong!!!";
        return ;
    }

    // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
    double chi2 = graph->activeChi2();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    int iterations = graph->optimize(_MaxIterationTimes);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double tcost= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    LOG(INFO) << std::endl << "done" << std::endl
              << "iterations: " << iterations << std::endl
              << "chi2: (before)" << chi2 << " -> (after)" <<  graph->activeChi2() << std::endl
              << "time: " << tcost << "[sec]" << std::endl;

    LOG(INFO) << "Optimization: chi2: " << chi2 << " -> " << graph->activeChi2() << std::endl;

}

ulong RobustOptimizer::getNodeSize() {

    return graph->vertices().size();
}

ulong RobustOptimizer::getEdgeSize() {

    return graph->edges().size();
}

bool RobustOptimizer::removeSE3Node(g2o::VertexSE3 *node) {

    bool nodeDone = graph->removeVertex(node);
    if(!nodeDone)
        LOG(ERROR) << "error in remove node" << std::endl;
    bool edgeDone = true;
    auto it = node->edges().begin(), itend = node->edges().end();
    for(; it != itend; ++it){
        if(!graph->removeEdge(*it)){
            edgeDone = false;
            LOG(ERROR) << "error in remove edges" << std::endl;
            break;
        }
    }
    return edgeDone&nodeDone;

}

Matrix6d RobustOptimizer::calcInfoMat(Matrix6d cov) {

    Matrix6d m = Matrix6d::Identity(6, 6);

    m = cov.inverse();

    return m;

}

Matrix6d RobustOptimizer::calcInfoMat(Vector6d cov) {

    Matrix6d m = Matrix6d::Identity(6, 6);

    for(int i = 0; i < 6; ++i) {
        m(i, i) = 1.0 / (cov(i));
    }

    return m;

}

Eigen::Isometry3d RobustOptimizer::sample_noise_from_se3(Vector6d& cov ) {

    double nx = g2o::Sampler::gaussRand(0., cov(0));
    double ny = g2o::Sampler::gaussRand(0., cov(1));
    double nz = g2o::Sampler::gaussRand(0., cov(2));

    double nroll = g2o::Sampler::gaussRand(0., cov(3));
    double npitch = g2o::Sampler::gaussRand(0., cov(4));
    double nyaw = g2o::Sampler::gaussRand(0., cov(5));

    Eigen::AngleAxisd aa(Eigen::AngleAxisd(nyaw, Eigen::Vector3d::UnitZ())*
                         Eigen::AngleAxisd(nroll, Eigen::Vector3d::UnitX())*
                         Eigen::AngleAxisd(npitch, Eigen::Vector3d::UnitY()));

    Eigen::Isometry3d retval = Eigen::Isometry3d::Identity();
    retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
    retval.translation() = Eigen::Vector3d(nx, ny, nz);

    return retval;
}

void RobustOptimizer::RemoveOutliers( int num_e){
    int n = 0;
    for(auto e:Myedges) {
        n++;
        if (e->chi2() > 2) {
            e->setLevel(1);
            LOG(INFO) << "edge" << n << "  is an outlier" << endl;
        }
        e->setRobustKernel(0);
    }
    LOG(INFO)<<endl<< "Finish removing outliers"<<endl<<"---- round2 ----"<<endl;
    return ;
}

