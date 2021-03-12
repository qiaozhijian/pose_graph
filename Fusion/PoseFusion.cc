#include "include/PoseFusion.h"
#include <list>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace ORB_SLAM3{
    PoseFusion::PoseFusion():
    {
        mnId = 0;
        CameraId = 0;
        IMUId = 0;
        bmatch = false;
        bsame = false;
        mbFailInterpolation = false;
        mlCameraDataSet.clear();
        mlbmatch.clear();
        mlNewFrame.clear();
        mlImuDataSet.clear();
        mlKeyFrames.clear();
        mlSE3Nodes.clear();
        optimizer = new RobustOptimizer(false, 100);
    }

    void PoseFusion::InsertCameraData(CameraData* pCameraData){
        mlCameraDataSet.push_back(pCameraData);
        mlNewFrame.push_back(pCameraData);
        CameraId ++;
    }

    void PoseFusion::InsertIMUData(ImuData* pImuData){
        mlImuDataSet.push_back(pImuData);
        IMUId ++;
    }
//*****************************************************************
    void PoseFusion::Run(){
        if(CheckNewFrames()){
            cout<<"Process Frame "<<mnId<<endl;
            mnId++;
            ProcessNewFrame();
            cout<<"---Pose Graph Optimization---"<<endl;
            if("检测到闭环"){ //TODO 闭环检测
                PoseGraphOptimization(); //执行图优化，优化关键帧位姿
            }
        }
    }
//*****************************************************************
    bool PoseFusion::CheckNewFrames(){
        return (!CameraDataSet.empty());
    }

    void PoseFusion::ProcessNewFrame(){
        mpCameraData = mlNewFrame.front();
        mlNewFrame.pop_front();
        //若是关键帧则保存
        if(mpCameraData.isKF()){
            mlKeyFrames.push_back(pCameraData)
        }
        //若相机LOST则利用IMU插值获取新位姿
        if(mpCameraData.NeedIMU()){
            IMUItor = SelectIMUId(); //选择用于插值的IMU帧
            mlbmatch.push_back(bmatch); //是否找到可用的IMU帧
            if(!bmatch) //没找到则报错返回
                // TODO 返回后怎样
                cerr<<"No Match IMU Found !!!"<<endl;
                mbFailInterpolation = true;
                return;
            if(!bsame){ //若相机和IMU时间戳相同则无需插值，不同则插值
                Quaterniond newQuaterniond = Interpolation(IMUItor);
                cv::Mat newPose = newQuaterniond.toRotationMatrix();
                mpCameraData.NewPose(newPose);
            }
            return;
        }
    }

    //选取用于插值的IMU帧
    std::list<ImuData *>::iterator PoseFusion::SelectIMUId(){
        double FrameTime = mpCameraData.GetTimeStamp(); //获取相机帧时间戳
        std::list<ImuData *>::iterator t_itor;
        for(std::list<ImuData *>::iterator itor = mlImuDataSet.begin(), itor != mlImuDataSet.end(), itor++){
            //如果已经遍历到了最后一帧IMU，相机与IMU时间戳相等则成功，不相等则报错
            if(itor == mlImuDataSet.end()){
                if(itor.GetTime() != FrameTime){
                    std::cerr<<"no matched IMU Data"<<endl;
                    return itor;
                }else{
                    std::cerr<<"matched!"<<endl;
                    bmatch = true;
                    bsame = true;
                    return itor;
                }
            }

            t_itor = itor + 1;
            time1 = itor.GetTime();
            time2 = t_itor.GetTime();
            if(time1 <= FrameTime && time2 >= FrameTime){
                std::cerr<<"matched!"<<endl;
                bmatch = true;
                if(time1 == FrameTime || time2 == FrameTime)
                    bsame = true;
                return itor;
            }
        }
    }

    //IMU插值
    Quaterniond PoseFusion::Interpolation(std::list<ImuData *>::iterator ImuItor)
    {
        // 四元数球面线性插值简化方法：v'=v1*cos(theta)' + v⊥*sin(theta)'
        std::cerr<<"Interpolation..."<<endl;
        Quaterniond q_slerp;
        std::list<ImuData *>::iterator tImuItor = ImuItor + 1;
        double t = (mpCameraData.GetTimeStamp()-ImuItor.GetTime())*1e-9;
        Quaterniond q1 = ImuItor.GetQuaterniond();
        Quaterniond q2 = tImuItor.GetQuaterniond();

        float cos_q1q2=q1.w()*q2.w()+q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z();
        //判断夹角
        if(cos_q1q2<0.0f){
            q1.w()=-1*q1.w();
            q1.x()=-1*q1.x();
            q1.y()=-1*q1.y();
            q1.z()=-1*q1.z();
            //cos_q1q2=-1*cos_q1q2;
            cos_q1q2=q1.w()*q2.w()+q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z();
        }
        float k1,k2;
        if(cos_q1q2>0.9995f){
            k1=1.0f-t;
            k2=t;
        }else{
            float sin_q1q2=sqrt(1.0f-cos_q1q2*cos_q1q2);
            float q1q2=atan2(sin_q1q2,cos_q1q2);
            k1=sin((1.0f-t)*q1q2)/sin_q1q2;
            k2=sin(t*q1q2)/sin_q1q2;
        }

        q_slerp.w()=k1*q1.w()+k2*q2.w();
        q_slerp.x()=k1*q1.x()+k2*q2.x();
        q_slerp.y()=k1*q1.y()+k2*q2.y();
        q_slerp.z()=k1*q1.z()+k2*q2.z();
        return q_slerp;
    }

    void PoseFusion::PoseGraphOptimization(){
        int i = 0;
        const Eigen::Isometry3d Tcw;
        int KFid;
        bool isFixed = false;
        ImuData IMUFrame;
        CameraData CameraFrame;
        Eigen::Matrix<double, 3, 3> IMURotation;
        //为每一帧关键帧设置顶点和边
        for(std::list<CameraData *>::iterator itor = mlKeyFrames.begin(), itor != mlKeyFrames.end(), itor++){
             //获取相机位姿
            Tcw = itor.GetRelativePose();
            Tcw = Eigen::Isometry3d(Tcw);
            KFid = i;
            //设置第一个顶点为固定顶点
            if(i == 0) {
                isFixed = true;
                g2o::VertexSE3* v1 = optimizer.addSE3Node(Tcw,KFid,isFixed);
                itor.Fixed(isFixed); //保存顶点状态(Fixed)
                isFixed = false;
            }
            else
            {
                g2o::VertexSE3* v1 = optimizer.addSE3Node(Tcw,KFid,isFixed);
                itor.Fixed(isFixed); //保存顶点状态(Fixed)
            }

            mlSE3Nodes.pushback(v1);
            //获取IMU位姿
            IMUFrame = mlImuDataSet.get(i);
            Eigen::Isometry3d IMUPose(IMUFrame.GetPose());
            //设置协方差矩阵
            Matrix6d cov = Matrix6d::Identity(6, 6);
            if(itor.NeedIMU())
                cov = 1e10*cov;
            //求信息矩阵
            Matrix6d info = RobustOptimizer::calcInfoMat(cov);
            //添加一元边
            optimizer.addSE3Edge(v1, IMUPose, info, 0);
            i++;
        }
        //开始求解
        optimizer.runOptimizer(0);
        //更新优化后的位姿
        i = 0;
        for(std::list<g2o::VertexSE3 *>::iterator itor = mlSE3Nodes.begin(), itor != mlSE3Nodes.end(), itor++){
            CameraFrame = mlKeyFrames.get(i);
            Eigen::Isometry3d newPose = itor->estimate();
            newpose = cv::Mat(newPose);
            CameraFrame.NewPose(newpose);
            i++;
        }
    }





}