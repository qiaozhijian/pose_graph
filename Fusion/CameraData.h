#ifndef CAMERADATA_H
#define CAMERADATA_H

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
    class CameraFrame{
    public:
        CameraData(Frame *pFrame);
        /*
        void Save();
        void SaveKFstate();
        void SaveTimeStamp();
        void SavePose();
         */
        void SetKeyFrame();
        void IsLost(bool State);
        void SaveMapPoints(Map* pMap);
        void NewPose(cv::Mat newTcw);
        void SaveCameraId(int Id)
        void Fixed(bool fixed);

        bool isKF();
        double GetTimeStamp();
        cv::Mat GetAbsolutePose();
        cv::Mat GetRelativePose();
        bool NeedIMU();
        vector<MapPoint *> GetMapPoints();
        int GetCameraId();
        bool isFixed();



    protected:
        /*
        list<bool> mlbisKF;
        list<bool> mlbLost;
        list<cv::Mat> mlRelativeCameraPoses;//相对位姿
        list<KeyFrame *> mlpReferenceKFs;//参考关键帧
        list<cv::Mat> mlAbsolutePose; //绝对位姿
        list<double> mlCameraTimeStamp;
        */
        vector<MapPoint *> mspAllMapPoints;
        int mnId;
        cv::Mat Tcr;
        cv::Mat Tcw;
        bool mbisKeyFrame;
        bool mbislost;
        bool mbisFixed;
    };
};
#endif // CAMERADATA_H