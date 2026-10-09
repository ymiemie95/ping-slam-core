//
// Created by auvatjacobs on 30.11.21.
// Modified: 新增 IMU 三轴安装旋转 (rotationBodyImu) 支持竖直/倒装 IMU
//

#ifndef UNDERWATERSLAM_EKFDVL_H
#define UNDERWATERSLAM_EKFDVL_H

#include <deque>
//#include "../src/slamTools/generalHelpfulTools.h"
#include "generalHelpfulTools.h"
#include "pose.h"
//#include "../src/slamTools/slamToolsRos.h"
//asynchronous EKF with reset of POS correction


class ekfClassDVL {
public:
    ekfClassDVL(rclcpp::Time timeRos) {
        this->stateOfEKF.position = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.rotation = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.velocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.angleVelocity = Eigen::Vector3f(0, 0, 0);
        this->stateOfEKF.timeLastPrediction = timeRos;

        this->processNoise = Eigen::MatrixXd::Identity(12, 12);
        this->processNoise(0, 0) = 0.02;//x
        this->processNoise(1, 1) = 0.02;//y
        this->processNoise(2, 2) = 0.005;//z
        this->processNoise(3, 3) = 0.005;//vx
        this->processNoise(4, 4) = 0.005;//vy
        this->processNoise(5, 5) = 0.01;//vz
        this->processNoise(6, 6) = 0.01;//r
        this->processNoise(7, 7) = 0.01;//p
        this->processNoise(8, 8) = 0.02;//y
        this->processNoise(9, 9) = 0.001;//vr
        this->processNoise(10, 10) = 0.001;//vp
        this->processNoise(11, 11) = 0.001;//vy

        this->measurementNoiseDepth = Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDepth(2, 2) = 0.5;//z

        this->measurementNoiseDVL = Eigen::MatrixXd::Identity(12, 12);
        this->measurementNoiseDVL(3, 3) = 0.2;//vx
        this->measurementNoiseDVL(4, 4) = 0.2;//vy
        this->measurementNoiseDVL(5, 5) = 0.2;//vz


        this->measurementImuVelocity = Eigen::MatrixXd::Identity(12, 12);
        this->measurementImuVelocity(6, 6) = 100.0;//r
        this->measurementImuVelocity(7, 7) = 100.0;//p
        this->measurementImuVelocity(9, 9) = 0.1;//vr
        this->measurementImuVelocity(10, 10) = 0.1;//vp
        this->measurementImuVelocity(11, 11) = 0.1;//vy



        this->stateOfEKF.covariance = processNoise;

        this->recentPoses.push_back(stateOfEKF);
    }

    // [新增] 设置 IMU 安装旋转 (IMU系 -> 体系), 节点读参后调用一次
    // R_body_imu = Rz(yaw) * Ry(pitch) * Rx(roll), 与 URDF rpy 约定一致
    void setImuMountRotation(double rollMount, double pitchMount, double yawMount);

    void
    predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,Eigen::Vector3d positionIMU, rclcpp::Time timeStamp);


    void updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL,Eigen::Vector3d positionDVL, rclcpp::Time timeStamp);

    // 原有体系版本: 输入必须已在体坐标系 (mount 旋转已完成)
    void updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                   Eigen::Quaterniond currentRotation, rclcpp::Time timeStamp);

    // [新增] 原始数据版本: 输入为 IMU 系原始加速度+角速度,
    // 内部完成 mount 旋转与 roll/pitch 提取后调用上面的体系版本
    void updateIMU(double xAccelImu, double yAccelImu, double zAccelImu,
                   double xAngularVelImu, double yAngularVelImu, double zAngularVelImu,
                   Eigen::Quaterniond currentRotation, rclcpp::Time timeStamp);

    void updateHeight(double depth, rclcpp::Time timeStamp);

    pose getState();

    Eigen::Quaterniond getRotationVector();

    Eigen::Quaterniond getRotationVectorWithoutYaw();


    Eigen::VectorXd innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H, Eigen::VectorXd currentStateBeforeUpdate);

private:
    pose stateOfEKF;
    //std::deque<edge> lastPositionDifferences;
    std::deque<pose> recentPoses;
    Eigen::MatrixXd processNoise, measurementNoiseDepth, measurementNoiseDVL, measurementImuVelocity, measurementNoiseSlam;
    rclcpp::Time lastUpdateTime;

    // [新增] IMU 安装旋转 (IMU系 -> 体系), 默认单位阵（不施加额外安装旋转）
    Eigen::Matrix3d rotationBodyImu = Eigen::Matrix3d::Identity();
};


#endif //UNDERWATERSLAM_EKFDVL_H