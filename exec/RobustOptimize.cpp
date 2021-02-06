//
// Created by zmy on 2021/2/6.
// 两段优化，去除外点
//
#include <iostream>
#include "glog/logging.h"
#include "global_defination/global_defination.h"
#include "RobustOptimizer.h"


using namespace std;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;

    RobustOptimizer optimizer(false, 100);

    //    设定初始位姿
    Eigen::Isometry3d groundTruth = Eigen::Isometry3d::Identity();

    Vector6d noise;
    Vector6d morenoise;
    int n = 40;
    double transNoise = 0.1 ;
    double rotNoise = 0.01 ;
    double moretransNoise = transNoise*n;
    double morerotNoise = rotNoise*n;
    noise << transNoise, transNoise, transNoise, rotNoise, rotNoise, rotNoise;
    morenoise << moretransNoise, moretransNoise, moretransNoise, morerotNoise, morerotNoise, morerotNoise;
    RobustOptimizer::sample_noise_from_se3(noise);
    RobustOptimizer::sample_noise_from_se3(morenoise);
    int num_sensors = 9;
    Eigen::Isometry3d initPose = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d imuPose = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d cameraPose = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d lidarPose = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d sensorPose1 = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d sensorPose2 = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d sensorPose3 = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d sensorPose4 = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;
    Eigen::Isometry3d sensorPose5 = RobustOptimizer::sample_noise_from_se3(morenoise) * groundTruth;
    Eigen::Isometry3d sensorPose6 = RobustOptimizer::sample_noise_from_se3(noise) * groundTruth;

    // g2o::VertexSE3* v1 = optimizer.addSE3Node(groundTruth, 0, false);
    g2o::VertexSE3 *v1 = optimizer.addSE3Node(initPose, 0, false);

    Matrix6d info = RobustOptimizer::calcInfoMat(noise);
    optimizer.addSE3Edge(v1, imuPose, info, 0);
    optimizer.addSE3Edge(v1, cameraPose, info, 0);
    optimizer.addSE3Edge(v1, lidarPose, info, 0);
    optimizer.addSE3Edge(v1, sensorPose1, info, 0);
    optimizer.addSE3Edge(v1, sensorPose2, info, 0);
    optimizer.addSE3Edge(v1, sensorPose3, info, 0);
    optimizer.addSE3Edge(v1, sensorPose4, info, 0);
    optimizer.addSE3Edge(v1, sensorPose5, info, 0);
    optimizer.addSE3Edge(v1, sensorPose6, info, 0);


    Eigen::Isometry3d initnodePose = v1->estimate();
    LOG(INFO) << "Before optimize, \n" << initnodePose.matrix() << endl;
    optimizer.runOptimizer(0);
    Eigen::Isometry3d nodePose = v1->estimate();
    LOG(INFO) << "After optimize, \n" << nodePose.matrix() << endl;
    optimizer.RemoveOutliers(num_sensors);
    optimizer.runOptimizer(0);
  }

