//
// Created by qzj on 2021/2/4.
//
#include <iostream>
#include "glog/logging.h"
#include "global_defination/global_defination.h"
#include "optimizer.h"

using namespace std;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;

    Optimizer optimizer(false, 100);
    
//    设定初始位姿
    Eigen::Isometry3d groundTruth = Eigen::Isometry3d::Identity();

    Vector6d noise;
    double transNoise = 0.1;
    double rotNoise = 0.01;
    noise << transNoise,transNoise,transNoise,rotNoise,rotNoise,rotNoise;
    Optimizer::sample_noise_from_se3(noise);

    Eigen::Isometry3d initPose = Optimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d imuPose = Optimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d cameraPose = Optimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d lidarPose = Optimizer::sample_noise_from_se3(noise) * groundTruth;

    //g2o::VertexSE3* v1 = optimizer.addSE3Node(groundTruth, 0, false);
    g2o::VertexSE3* v1 = optimizer.addSE3Node(initPose, 0, false);
/*
    g2o::SE3Quat INITPose = g2o::internal::toSE3Quat(initPose);
    g2o::VertexSE3Expmap* v1 = optimizer.addSE3ExpNode(INITPose, 0, false);
*/
    Matrix6d info = Optimizer::calcInfoMat(noise);

    optimizer.addSE3Edge(v1, imuPose, info, 0);
    optimizer.addSE3Edge(v1, cameraPose, info, 0);
    optimizer.addSE3Edge(v1, lidarPose, info, 0);

/*
    optimizer.addSE3ExpEdge(v1, imuPose, info, 0);
    optimizer.addSE3ExpEdge(v1, cameraPose, info, 0);
    optimizer.addSE3ExpEdge(v1, lidarPose, info, 0);
*/
    Eigen::Isometry3d initnodePose = v1->estimate();
    //G2O_TYPES_SLAM3D_API Isometry3D fromSE3Quat(const SE3Quat& t);
    LOG(INFO)<< "Before optimize, \n" << initnodePose.matrix() <<endl;

    optimizer.runOptimizer(0);

    Eigen::Isometry3d nodePose = v1->estimate();

    LOG(INFO)<< "After optimize, \n" << nodePose.matrix() <<endl;
}
