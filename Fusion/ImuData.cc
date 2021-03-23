#include "ImuData.h"
#include <list>
#include <iostream>

using namespace std;

namespace ORB_SLAM3{
    ImuData(Frame* pFrame, double IMUTime):
        mpFrame(pFrame){

    }
/*
    void ImuData::Save(){
        SavePose();
        SaveTime();
    }

    void ImuData::SavePose(){
        std::cerr<<"Save Pose"<<endl;
        IMUPose = mpFrame.GetImuPose();
        mlIMUPoses.push_back(IMUPose);
    }

    void ImuData::SaveTime(){
        std::cerr<<"Save Time"<<endl;
        mlIMUTime.push_back(IMUTime);
    }
*/
//****************************************************************************

    void IMUData::SaveIMUId(int Id){
        mnId = Id;
    }

    cv::Mat ImuData::GetPose(){
        IMUPose = mpFrame.GetImuPose();
        return IMUPose;
    }

    cv::Mat ImuData::GetPosition() {
        cv::Mat ImuPosition = mpFrame.GetImuPosition();
        return ImuPosition;
    }

    cv::Mat ImuData::GetRotation() {
        cv::Mat ImuRotation = mpFrame.GetImuRotation();
        return ImuRotation;
    }

    double ImuData::GetTime(){
        return IMUTime;
    }

    Quaterniond ImuData::GetQuaterniond(){
        Quaterniond QPose(GetRotation());
        return QPose;
    }

    int CameraData::GetIMUId(){
        return mnId;
    }



}