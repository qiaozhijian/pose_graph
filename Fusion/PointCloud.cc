#include "include/PointCloud.h"
#include <list>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace ORB_SLAM3{
    PointCloud::PointCloud(double resolution_){
        this->resolution = resolution_; //分辨率，参考配置文件中设置为0.01
        voxel.setLeafSize( resolution, resolution, resolution);
        globalMap = boost::make_shared< PointCloud >( );
        viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    }

    void PointCloud::shutdown()
    {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloud::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
        unique_lock<mutex> lck(keyframeMutex);
        keyframes.push_back( kf );
        colorImgs.push_back( color.clone() );
        depthImgs.push_back( depth.clone() );

        keyFrameUpdated.notify_one();
    }

    pcl::PointCloud< PointCloud::PointT >::Ptr PointCloud::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        PointCloud::Ptr tmp( new PointCloud() );
        // point cloud is null ptr
        for ( int m=0; m<depth.rows; m+=3 )
        {
            for ( int n=0; n<depth.cols; n+=3 )
            {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d>10)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;

                p.b = color.ptr<uchar>(m)[n*3];
                p.g = color.ptr<uchar>(m)[n*3+1];
                p.r = color.ptr<uchar>(m)[n*3+2];

                tmp->points.push_back(p);
            }
        }

        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
        PointCloud::Ptr cloud(new PointCloud);
        pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
        cloud->is_dense = false;

        cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
        return cloud;
    }


    void PointCloud::viewer()
    {
        pcl::visualization::CloudViewer viewer("viewer");
        while(1)
        {
            {
                unique_lock<mutex> lck_shutdown( shutDownMutex );
                if (shutDownFlag)
                {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );
            }

            // keyframe is updated
            size_t N=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N = keyframes.size();
            }

            for ( size_t i=lastKeyframeSize; i<N ; i++ )
            {
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                *globalMap += *p;
            }
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( globalMap );
            voxel.filter( *tmp );
            globalMap->swap( *tmp );
            viewer.showCloud( globalMap );
            cout << "show global map, size=" << globalMap->points.size() << endl;
            lastKeyframeSize = N;
        }

}