#ifndef POINTCLOUD_H
#define POINTCLOUD_H

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
    public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud( double resolution_ );

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();
    void viewer();

    protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;

    bool shutDownFlag = false;
    mutex shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex keyFrameUpdateMutex;

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};
#endif // POINTCLOUD_H