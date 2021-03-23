#include "CameraData.h"
#include <list>
#include <iostream>

using namespace std;
namespace ORB_SLAM3{
    CameraData::CameraData(Frame *pFrame):
                            mpFrame(pFrame) {
        mbisKeyFrame = false;
        mbislost = false;
        mbisFixed = false;
        mspAllMapPoints.clear();
        Tcw = mpFrame.mTcw;
    }

/*
    void CameraData::Save(Map* pMap, eTrackingState mState){
        SaveKFstate();
        SaveTimeStamp();
        SavePose();
        SaveState(mState);
        SaveMapPoints(pMap);
    }

    void CameraData::SaveKFstate(){ //保存是否为关键帧
        std::cerr<<"SaveKFstate"<<endl;
        mlbisKF.push_back(mbisKeyFrame);
    }

    void CameraData::SaveTimeStamp(){ //保存时间戳
        std::cerr<<"SaveTimeStamp"<<endl;
        mlCameraTimeStamp.push_back(mpFrame.mTimeStamp);
    }

    void CameraData::SavePose(){ //保存(相对和绝对)位姿和参考关键帧
        std::cerr<<"SavePose"<<endl;
        cv::Mat Tcw = mpFrame.mTcw;
        mlRelativeCameraPoses.push_back(Tcw);
        mlpReferenceKFs.push_back(mpFrame.mpReferenceKF);
        Tcr = Tcw * mpFrame.mpReferenceKF->GetPoseInverse();
        mlAbsolutePose.push_back(Tcr);
    }
*/
    void CameraData::SetKeyFrame(){ //设为关键帧
        std::cerr<<"This is a Key Frame"<<endl
        mbisKeyFrame = true;
    }

    void CameraData::IsLost(bool State){ //相机帧是否丢失
        std::cerr<<"SaveState"<<endl;
        mbislost = State;
    }

    void CameraData::SaveMapPoints(Map* pMap){ //保存地图点
        std::cerr<<"SaveMapPoints"<<endl;
        mspAllMapPoints.insert(pMap.mspMapPoints) ;
    }

    void CameraData::NewPose(cv::Mat newTcw){
        std::cerr<<"Get New Pose From Imu"<<endl;
        Tcw = newTcw;
        Tcr = Tcw * mpFrame.mpReferenceKF->GetPoseInverse();
    }

    void CameraData::Fixed(bool fixed){
        mbisFixed = fixed;
    }

    void CameraData::SaveCameraId(int Id){
        mnId = Id;
    }

//***************************************************************
    bool CameraData::isKF(){ //是否为关键帧
        return mbisKeyFrame;
    }

    double CameraData::GetTimeStamp(){ //获取时间戳
        return mpFrame.mTimeStamp;
    }

    cv::Mat CameraData::GetAbsolutePose(){ //获取(绝对)位姿
        Tcr = Tcw * mpFrame.mpReferenceKF->GetPoseInverse();
        return Tcr;
    }

    cv::Mat CameraData::GetRelativePose(){ //获取(相对)位姿
        return Tcw;
    }

    bool CameraData::NeedIMU(){ //是否需要IMU(相机帧是否丢失)
        return mbislost;
    }

    vector<MapPoint *> CameraData::GetMapPoints(){ //获取地图点
        return vector<MapPoint *>(mspAllMapPoints.begin(),mspAllMapPoints.end())  ;
    }

    int CameraData::GetCameraId(){
        return mnId;
    }

    bool CameraData::isFixed(){
        return mbisFixed;
    }
}