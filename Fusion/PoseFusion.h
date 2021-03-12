#ifndef POSEFUSION_H
#define POSEFUSION_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <mutex>
#include <queue>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "Tracking.h"
#include "Convertor.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "include/sensor/ImuTypes.h"
#include "include/sensor/Odometer.h"
#include "include/sensor/imuProcess.h"
#include "CameraData.h"

namespace ORB_SLAM3{
    class Viewer;
    class KeyFrame;
    class FrameDrawer;
    class MapPoint;
    class Atlas;
    class Tracking;
    class LocalMapping;
    class LoopClosing;
    class System;
    class Map;
    class CameraData;
    class IMUData;
    class RobustOptimizer;
    class PoseFusion{
    public:
        PoseFusion();
        void InsertCameraData(CameraData* pCameraData);
        void InsertIMUData(ImuData* pImuData);
        void Run();
        bool CheckNewFrames();
        void ProcessNewFrame();
        std::list<ImuData *>::iterator SelectIMUId();
        Quaterniond Interpolation(std::list<ImuData *>::iterator ImuItor);
        void PoseGraphOptimization();

    protected:
        std::list<CameraData *> mlCameraDataSet;
        std::list<CameraData *> mlKeyFrames;
        std::list<CameraData *> mlNewFrame;
        std::list<ImuData *> mlImuDataSet;
        std::list<ImuData *> mlIMUKeyFrames;
        std::list<ImuData *>::iterator IMUItor;
        std::list<bool> mlbmatch;
        std::list<g2o::VertexSE3 *> mlSE3Nodes;
        CameraData * mpCameraData;
        int CameraId;
        int IMUId;
        bool bmatch;
        bool bsame;
        int mnId;
        bool mbFailInterpolation;
    };
};
#endif // POSEFUSION_H