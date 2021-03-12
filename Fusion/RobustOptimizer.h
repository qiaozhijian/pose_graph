
#ifndef ROBUSTOPTIMIZER_H
#define ROBUSTOPTIMIZER_H
#include <cmath>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include "g2o/core/robust_kernel_impl.h"
#include <g2o/types/sba/vertex_se3_expmap.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class RobustOptimizer {

public:

    RobustOptimizer(const bool& Verbose, const int& maxIteration);

    g2o::VertexSE3* addSE3Node(const Eigen::Isometry3d &pose, const int KFid, const bool& isFixed);

    void addSE3Edge(g2o::VertexSE3* &v1, g2o::VertexSE3* &v2, const Eigen::Isometry3d &relative_pose,
                    const Matrix6d &info_matrix, int level);

    void addSE3Edge(g2o::VertexSE3* &v1, const Eigen::Isometry3d &iso_pose,
                    const Matrix6d &info_matrix, int level);

    ulong getNodeSize();

    ulong getEdgeSize();

    void runOptimizer(int level=0);

    bool removeSE3Node(g2o::VertexSE3 *node);

    void addSE3PlaneEdge(g2o::VertexSE3* &v1, int level);

    static Matrix6d calcInfoMat(Matrix6d cov);

    static Matrix6d calcInfoMat(Vector6d cov);

    static Eigen::Isometry3d sample_noise_from_se3(Vector6d& cov );

    void RemoveOutliers(int num_e);

private:

    g2o::SparseOptimizer* graph;

    bool _Verbose;

    int _MaxIterationTimes;
};


#endif //ROBUSTOPTIMIZER_H
