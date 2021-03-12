#ifndef IMUDATA_H
#define IMUDATA_H

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
    class ImuData{
    public:
        ImuData(Frame* pFrame, double IMUTime);
        /*
        void Save();
        void SavePose();
        void SaveTime();
         */
        void SaveIMUId(int Id);
        cv::Mat GetPose();
        cv::Mat GetPosition();
        cv::Mat GetRotation();
        double GetTime();
        Quaterniond GetQuaterniond()
        int GetIMUId();

    protected:
        list<cv::Mat> mlIMUPoses;
        list<double> mlIMUTime;
        double IMUTime;
        cv::Mat IMUPose;
        int mnId;

    };

};
#endif // IMUDATA_H