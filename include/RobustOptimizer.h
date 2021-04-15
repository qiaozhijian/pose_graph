//
// Created by qzj on 2021/2/4.
//

#ifndef POSE_GRAPH_ROBUST_OPTIMIZER_H
#define POSE_GRAPH_ROBUST_OPTIMIZER_H
#include <cmath>
#include "CommonFunc.h"
#include "optimizer.h"

class RobustOptimizer : public Optimizer {

public:

  RobustOptimizer(const bool& verbose, const int& maxIteration);

  void runOptimizer(int level=0);

  void RemoveOutliers();

};


#endif //POSE_GRAPH_OPTIMIZER_H
