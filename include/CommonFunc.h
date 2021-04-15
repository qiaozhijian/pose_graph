//
// Created on 19-3-27.
//

#ifndef PROJECT_COMMONFUNC_H
#define PROJECT_COMMONFUNC_H

#include <Eigen/Geometry>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

template<typename T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 3, 3> skew_matrix = Eigen::Matrix<T, 3, 3>::Identity();

    skew_matrix<< 0, -axis(2,0), axis(1,0),
            axis(2,0), 0, -axis(0,0),
            -axis(1,0), axis(0,0), 0;

    return skew_matrix;
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> dcm2rpy(const Eigen::Matrix<T, 3, 3> &R) {
    Eigen::Matrix<T, 3, 1> rpy;
    rpy[1] = atan2(-R(2, 0), sqrt(pow(R(0, 0), 2) + pow(R(1, 0), 2)));
    if (fabs(rpy[1] - M_PI / 2) < 0.00001) {
        rpy[2] = 0;
        rpy[0] = -atan2(R(0, 1), R(1, 1));
    } else {
        if (fabs(rpy[1] + M_PI / 2) < 0.00001) {
            rpy[2] = 0;
            rpy[0] = -atan2(R(0, 1), R(1, 1));
        } else {
            rpy[2] = atan2(R(1, 0) / cos(rpy[1]), R(0, 0) / cos(rpy[1]));
            rpy[0] = atan2(R(2, 1) / cos(rpy[1]), R(2, 2) / cos(rpy[1]));
        }
    }
    return rpy;
}

inline void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

inline Eigen::Matrix3d rpy2dcm(const Eigen::Vector3d &rpy)//(yaw)
{
    Eigen::Matrix3d R1;
    R1(0, 0) = 1.0;
    R1(0, 1) = 0.0;
    R1(0, 2) = 0.0;
    R1(1, 0) = 0.0;
    R1(1, 1) = cos(rpy[0]);
    R1(1, 2) = -sin(rpy[0]);
    R1(2, 0) = 0.0;
    R1(2, 1) = -R1(1, 2);
    R1(2, 2) = R1(1, 1);

    Eigen::Matrix3d R2;
    R2(0, 0) = cos(rpy[1]);
    R2(0, 1) = 0.0;
    R2(0, 2) = sin(rpy[1]);
    R2(1, 0) = 0.0;
    R2(1, 1) = 1.0;
    R2(1, 2) = 0.0;
    R2(2, 0) = -R2(0, 2);
    R2(2, 1) = 0.0;
    R2(2, 2) = R2(0, 0);

    Eigen::Matrix3d R3;
    R3(0, 0) = cos(rpy[2]);
    R3(0, 1) = -sin(rpy[2]);
    R3(0, 2) = 0.0;
    R3(1, 0) = -R3(0, 1);
    R3(1, 1) = R3(0, 0);
    R3(1, 2) = 0.0;
    R3(2, 0) = 0.0;
    R3(2, 1) = 0.0;
    R3(2, 2) = 1.0;

    return R3 * R2 * R1;
}


inline Vector6d toVector6d(Eigen::Matrix4d &matT) {

    Eigen::Matrix3d rot = matT.block(0, 0, 3, 3);
    Eigen::Vector3d angle = dcm2rpy(rot);
    Eigen::Vector3d trans(matT(0, 3), matT(1, 3), matT(2, 3));
    Vector6d pose;
    pose(0) = trans(0);
    pose(1) = trans(1);
    pose(2) = trans(2);
    pose(3) = angle(0) * 180 / M_PI;
    pose(4) = angle(1) * 180 / M_PI;
    pose(5) = angle(2) * 180 / M_PI;
    return pose;
}

inline Eigen::Matrix3f Euler2DCM(float roll, float pitch, float yaw) {
//        输入的角度是弧度制
    Eigen::Vector3f ea(yaw, pitch, roll);
    //欧拉角转换为旋转矩阵
    Eigen::Matrix3f rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitX());
    return rotation_matrix3;
}

inline Eigen::Matrix4f toEigen4fInverse(const Eigen::Matrix4f &Tcw) {
    Eigen::Matrix3f Rcw = Tcw.block(0, 0, 3, 3);
    Eigen::Vector3f tcw = Tcw.block(0, 3, 3, 1);
    Eigen::Matrix3f Rwc = Rcw.transpose();
    Eigen::Vector3f twc = -Rwc * tcw;

    Eigen::Matrix4f Twc = Eigen::Matrix4f::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = twc;

    return Twc;
}

inline Eigen::Matrix4d toEigen4dInverse(const Eigen::Matrix4d &Tcw) {
    Eigen::Matrix3d Rcw = Tcw.block(0, 0, 3, 3);
    Eigen::Vector3d tcw = Tcw.block(0, 3, 3, 1);
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Eigen::Vector3d twc = -Rwc * tcw;

    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = twc;

    return Twc;
}

inline Eigen::Isometry3d Mat4d2Iso3d(Eigen::Matrix4d &mat) {

    Eigen::Isometry3d matIso;
    matIso.setIdentity();

    Eigen::Matrix3d rotMat = mat.block(0, 0, 3, 3);
    Eigen::Vector3d trans(mat(0, 3), mat(1, 3), mat(2, 3));

    matIso.translate(trans);
    matIso.rotate(rotMat);

    return matIso;
}

inline Eigen::Matrix4d Iso3d2Mat4d(Eigen::Isometry3d &iso) {

    Eigen::Matrix3d rot = iso.rotation();
    Eigen::Vector3d trans = iso.translation();
    Eigen::Matrix4d mat;
    mat.setIdentity();
    mat.block(0, 0, 3, 3) = rot;
    mat(0, 3) = trans(0);
    mat(1, 3) = trans(1);
    mat(2, 3) = trans(2);

    return mat;

}


inline void PrintDeltaPose(Eigen::Isometry3d &T1, Eigen::Isometry3d &T2) {

    Eigen::Isometry3d deltaT = T1.inverse() * T2;

    Eigen::Matrix3d rot = deltaT.rotation();
    Eigen::Vector3d trans = deltaT.translation();
    Eigen::Vector3d rpy = dcm2rpy(rot);
    double theta = sqrt(rpy(0)*rpy(0)+rpy(1)*rpy(1)+rpy(2)*rpy(2));
    double transSum = sqrt(trans(0)*trans(0)+trans(1)*trans(1)+trans(2)*trans(2));

    printf("Delra theta %f, trans %f", theta, transSum);

}

#endif //PROJECT_COMMONFUNC_H
