#include "Optimizer.h"
#include <ros/ros.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <boost/format.hpp>

using namespace PoseOptimizer {

  Optimizer::Optimizer() {}

  bool Optimizer::Setup(const bool &Verbose, const int &maxIteration,
                        bool mode) {

    graph = new g2o::SparseOptimizer();
    _Verbose = Verbose;
    _MaxIterationTimes = maxIteration;

    std::string g2o_solver_name = "pose optimizer";
    std::cout << "construct solver... " << std::endl;

    // auto linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>(); auto blockSolver = new g2o::BlockSolverX(linearSolver); auto solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

    typedef g2o::BlockSolverX BlockSolverType;
    typedef g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>
        LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    // 设置求解器
    graph->setAlgorithm(solver);
    //打开调试输出
    graph->setVerbose(_Verbose);

    if (!graph->solver()) {
      std::cerr << std::endl;
      std::cerr << "error : failed to allocate solver!!" << std::endl;
      return false;
    }

    //有些程序加了这三句话，但没有给出原因
    g2o::ParameterSE3Offset *cameraOffset = new g2o::ParameterSE3Offset;
    cameraOffset->setId(0);
    graph->addParameter(cameraOffset);

    _InLocalizationMode = mode;

    std::cout << "Allocating solver done!" << std::endl;

    return true;
  }
  //加雷达位姿的节点
  g2o::VertexSE3 *Optimizer::addLidarSE3Node(
      const Eigen::Isometry3d &pose, const int KFid, const bool &isFixed) {

    auto vertex(new g2o::VertexSE3());
    g2o::SE3Quat pose_ = g2o::internal::toSE3Quat(pose);
    vertex->setId(KFid);
    vertex->setEstimate(pose_);
    vertex->setFixed(isFixed);
    graph->addVertex(vertex);

    return vertex;
  }

  void Optimizer::addPlaneMotionConstrain(g2o::VertexSE3 * &v1, int level) {

    Eigen::Isometry3d pose = v1->estimate();

    Eigen::Matrix3d rot = pose.rotation();
    Eigen::Vector3d rpy = laser_slam::dcm2rpy(rot);

    // fix pitch and raw to zero, only yaw remains
    double yaw = rpy[2];
    Eigen::Matrix3d planeRot;
    planeRot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    // fix height to zero
    Eigen::Vector3d trans = pose.translation();
    trans[2] = 0;

    g2o::SE3Quat planePose;
    planePose.setRotation(Eigen::Quaterniond(planeRot));
    planePose.setTranslation(trans);

    // info xyz，欧拉角的信息矩阵  把zrp设得很小，xyy设得很大
    Eigen::MatrixXd info = Eigen::MatrixXd::Identity(6, 6);
    info(0, 0) /= pow(1e6, 2);  // x
    info(1, 1) /= pow(1e6, 2);  // y
    info(2, 2) /= pow(1e-6, 2); // z
    info(3, 3) /= pow(1e-6, 2); // r
    info(4, 4) /= pow(1e-6, 2); // p
    info(5, 5) /= pow(1, 2);    // y

    //欧式变换矩阵，虽然写的是3d，但其实是4*4的矩阵
    Eigen::Isometry3d iso_pose = planePose;
    // EdgeSE3Prior:	Provides a prior for a 3d pose vertex. Again the measurement is represented by an Isometry3 matrix. 是个一元边
    //这里是为了只对xyy分量进行约束，其他保持不变（好像是这样的）
    auto planeConstraint = new g2o::EdgeSE3Prior();

    planeConstraint->setInformation(info);
    planeConstraint->setMeasurement(iso_pose);
    planeConstraint->setLevel(level);
    planeConstraint->setVertex(0, v1);
    planeConstraint->setParameterId(0, 0);
    graph->addEdge(planeConstraint);
  }

  void Optimizer::addSE3Edge(g2o::VertexSE3 * &v1,
                             const Eigen::Isometry3d &relative_pose,
                             const Eigen::MatrixXd &info_matrix, int level) {

    auto edge = new g2o::EdgeSE3();
    edge->setMeasurement(relative_pose);
    edge->setInformation(info_matrix);
    edge->setVertex(0, v1);
    // edge->setVertex(1, v2);
    edge->setLevel(level);
    graph->addEdge(edge);
  }

  void Optimizer::runOptimizer(int level) {

    if (graph->edges().size() < 10) {
      return;
    }

    LOG(INFO) << std::endl
              << "--- pose graph optimization ---" << std::endl
              << "nodes: " << graph->vertices().size() << "   "
              << "edges: " << graph->edges().size() << std::endl
              << "optimizing... ";

    if (!graph->initializeOptimization(level)) {
      LOG(ERROR) << "Graph initialize Optimization Wrong!!!";
      return;
    }

    double chi2 = graph->activeChi2();

    if (chi2 < 10e3 && !_InLocalizationMode) {
      LOG(INFO) << " is too small, cancel optimization" << std::endl;
      return;
    }

    auto t1 = ros::Time::now();
    int iterations = graph->optimize(_MaxIterationTimes);

    auto t2 = ros::Time::now();
    LOG(INFO) << std::endl
              << "done" << std::endl
              << "iterations: " << iterations << std::endl
              << "chi2: (before)" << chi2 << " -> (after)"
              << graph->activeChi2() << std::endl
              << "time: " << boost::format("%.3f") % (t2 - t1).toSec()
              << "[sec]" << std::endl;

    std::cout << "Optimization: chi2: " << chi2 << " -> " << graph->activeChi2()
              << std::endl;
  }

  int main(void) {
    _optimizer = new PoseOptimizer::Optimizer();
    _optimizer->Setup(_verbose, _maxIterationTimes, true);
    // add node
    node1 =
        _optimizer->addLidarSE3Node(PoseOptimizer::Mat4d2Iso3d(pose), 0, false);
    _optimizer->addPlaneMotionConstrain(node1, 1);

    // add edge
    N = 30;
    for (i = 1; i <= N; i++) {
      Eigen::MatrixXd info = _optimizer->calcInfoMat(
          transform, _useFixedParam, _stddev_trans * 2, _stddev_rotate * 2);
      _optimizer->addSE3Edge(node1, Mat4d2Iso3d(transform), info, i);
    }

    // optimize
    _optimizer->runOptimizer(1);
  }
} //namespace PoseOptimizer
