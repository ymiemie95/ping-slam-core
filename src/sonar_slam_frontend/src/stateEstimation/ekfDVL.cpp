//
// Created by jurobotics on 13.09.21.
// Modified: 新增 IMU 三轴安装旋转 rotationBodyImu (竖直/倒装 IMU 支持)
//   [改动1] setImuMountRotation(): 由 rollMountIMU/pitchMountIMU/yawMountIMU 构造 R_body_imu
//   [改动2] predictionImu(): 加速度先经 R_body_imu 旋入体系
//   [改动3] updateIMU() 新增重载: 接收 IMU 系原始加速度+角速度,
//           内部旋转后从加速度提取 roll/pitch, 再调用原有体系版本
//   mount 角全为 0 时安装旋转为单位阵；此处不声明整套系统与上游数值等价。
//
#include "ekfDVL.h"

// [改动1] 设置 IMU 安装旋转 (IMU系 -> 体系), 节点读参后调用一次
// R_body_imu = Rz(yaw) * Ry(pitch) * Rx(roll), 与 URDF rpy 约定一致
void ekfClassDVL::setImuMountRotation(double rollMount, double pitchMount, double yawMount) {
    this->rotationBodyImu =
            (Eigen::AngleAxisd(yawMount, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitchMount, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rollMount, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void ekfClassDVL::predictionImu(double xAccel, double yAccel, double zAccel, Eigen::Quaterniond currentRotation,
                                Eigen::Vector3d positionIMU,
                                rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    // A-Matrix is identity, except for the entries of transition between velocity and position.(there time diff since last prediction)
    // update state
    // [改动2] IMU 系加速度先旋入体系 (mount 角为 0 时 rotationBodyImu 为单位阵, 行为不变)
    Eigen::Vector3d bodyAcceleration =
            this->rotationBodyImu * Eigen::Vector3d(xAccel, yAccel, zAccel);


    Eigen::Vector3d angularVelocity(currentStateBeforeUpdate[9], currentStateBeforeUpdate[10],
                                    currentStateBeforeUpdate[11]);

    Eigen::Vector3d bodyAccelerationReal = (bodyAcceleration +
                                            this->getRotationVector().inverse() * Eigen::Vector3d(0, 0, 9.81) -
                                            angularVelocity.cross(angularVelocity.cross(positionIMU)));


    // bodyAcceleration has to be changed to correct rotation(body acceleration)
    Eigen::Vector3d localAcceleration = this->getRotationVector() *
                                        bodyAccelerationReal;


    double timeDiff = (timeStamp - this->stateOfEKF.timeLastPrediction).seconds();

    if (timeDiff > 0.4 || timeDiff < 0) {
        timeDiff = 0.4;
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(12, 12);
    //state Transition Matrix
    A(0, 3) = timeDiff;
    A(1, 4) = timeDiff;
    A(2, 5) = timeDiff;
    A(6, 9) = timeDiff;
    A(7, 10) = timeDiff;
    A(8, 11) = timeDiff;
    Eigen::VectorXd state = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();
    Eigen::VectorXd inputMatrix = Eigen::VectorXd::Zero(12);
    inputMatrix(3) = localAcceleration(0) * timeDiff;
    inputMatrix(4) = localAcceleration(1) * timeDiff;
    inputMatrix(5) = localAcceleration(2) * timeDiff;
    state = A * state + inputMatrix;
    //look for wrap around
    if (state(8) > M_PI) {
        state(8) = state(8) - 2 * M_PI;
    }
    if (state(8) < -M_PI) {
        state(8) = state(8) + 2 * M_PI;
    }

    this->stateOfEKF.applyState(state);
    //update covariance
    this->stateOfEKF.covariance = A * this->stateOfEKF.covariance * A.transpose() + processNoise;
    this->stateOfEKF.timeLastPrediction = timeStamp;

}

// [改动3] 新增重载: 接收 IMU 系原始数据 (加速度 + 角速度),
// 内部完成 mount 旋转与 roll/pitch 提取, 再调用原有体系版本。
// 节点回调改为调用本函数, 不再自行提取 roll/pitch。
void ekfClassDVL::updateIMU(double xAccelImu, double yAccelImu, double zAccelImu,
                            double xAngularVelImu, double yAngularVelImu, double zAngularVelImu,
                            Eigen::Quaterniond currentRotation,
                            rclcpp::Time timeStamp) {
    Eigen::Vector3d accelBody =
            this->rotationBodyImu * Eigen::Vector3d(xAccelImu, yAccelImu, zAccelImu);
    Eigen::Vector3d gyroBody =
            this->rotationBodyImu * Eigen::Vector3d(xAngularVelImu, yAngularVelImu, zAngularVelImu);

    // NED 体系下由加速度计提取 roll/pitch:
    // 静止水平时 accelBody = (0, 0, -9.81) -> roll = 0, pitch = 0
    // (绕开姿态四元数的 ENU/NED 世界系约定问题, 低动态水池场景足够稳)
    double roll = std::atan2(-accelBody.y(), -accelBody.z());
    double pitch = std::atan2(accelBody.x(),
                              std::sqrt(accelBody.y() * accelBody.y() +
                                        accelBody.z() * accelBody.z()));

    this->updateIMU(roll, pitch, gyroBody.x(), gyroBody.y(), gyroBody.z(),
                    currentRotation, timeStamp);
}

void ekfClassDVL::updateIMU(double roll, double pitch, double xAngularVel, double yAngularVel, double zAngularVel,
                            Eigen::Quaterniond currentRotation,
                            rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    Eigen::VectorXd innovation;
    //change from system to body system
    Eigen::Vector3d velocityBodyAngular(xAngularVel, yAngularVel, zAngularVel);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalAngular = this->getRotationVectorWithoutYaw() * velocityBodyAngular;

    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(6) = roll;
    z(7) = pitch;
    z(9) = velocityLocalAngular(0);
    z(10) = velocityLocalAngular(1);
    z(11) = velocityLocalAngular(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(6, 6) = 1;
    H(7, 7) = 1;
    H(9, 9) = 1;
    H(10, 10) = 1;
    H(11, 11) = 1;
    if (roll > 0 && currentStateBeforeUpdate[6] < 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] + M_PI * 2;
    }
    if (roll < 0 && currentStateBeforeUpdate[6] > 0) {
        currentStateBeforeUpdate[6] = currentStateBeforeUpdate[6] - M_PI * 2;
    }
    innovation = this->innovationStateDiff(z, H, currentStateBeforeUpdate);//also called y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementImuVelocity;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = currentStateBeforeUpdate + K * innovation;

    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;

}

void
ekfClassDVL::updateDVL(double xVel, double yVel, double zVel, Eigen::Quaterniond rotationOfDVL,
                       Eigen::Vector3d positionDVL, rclcpp::Time timeStamp) {
    //for saving the current EKF pose difference in

    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();

    Eigen::Vector3d velocityBodyLinear(xVel, yVel, zVel);
    velocityBodyLinear = rotationOfDVL * velocityBodyLinear;
    //reduce velocity dependent on rotation and position of dvl
    Eigen::Vector3d angularVelocity(currentStateBeforeUpdate[9], currentStateBeforeUpdate[10],
                                    currentStateBeforeUpdate[11]);
    velocityBodyLinear = velocityBodyLinear - angularVelocity.cross(positionDVL);
    // velocityAngular has to be changed to correct rotation(world velocityAngular)
    Eigen::Vector3d velocityLocalLinear = this->getRotationVector() * velocityBodyLinear;

    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(3) = velocityLocalLinear(0);
    z(4) = velocityLocalLinear(1);
    z(5) = velocityLocalLinear(2);// removed because z is not important. velocityLocalLinear(2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(3, 3) = 1;
    H(4, 4) = 1;
    H(5, 5) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDVL;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

void
ekfClassDVL::updateHeight(double depth, rclcpp::Time timeStamp) {
    if (isnan(depth)) {
        depth = 0;
    }
    //for saving the current EKF pose difference in
    Eigen::VectorXd currentStateBeforeUpdate = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();


    Eigen::VectorXd innovation;
    Eigen::VectorXd z = Eigen::VectorXd::Zero(12);
    z(2) = depth;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12, 12);
    H(2, 2) = 1;
    innovation = z - H * this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel();//also y
    Eigen::MatrixXd S = H * this->stateOfEKF.covariance * H.transpose() + this->measurementNoiseDepth;
    Eigen::MatrixXd K = this->stateOfEKF.covariance * H.transpose() * S.inverse();
    Eigen::VectorXd newState = this->stateOfEKF.getStatexyzvxvyvzrpyrvelpvelyvel() + K * innovation;
    this->stateOfEKF.applyState(newState);
    this->stateOfEKF.covariance = (Eigen::MatrixXd::Identity(12, 12) - K * H) * this->stateOfEKF.covariance;
}

pose ekfClassDVL::getState() {
    return this->stateOfEKF;
}

Eigen::Quaterniond ekfClassDVL::getRotationVector() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    double rotationZ = this->stateOfEKF.rotation.z();

    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, rotationZ);
    return rotDiff;
}

Eigen::Quaterniond ekfClassDVL::getRotationVectorWithoutYaw() {
    double rotationX = this->stateOfEKF.rotation.x();
    double rotationY = this->stateOfEKF.rotation.y();
    Eigen::Quaterniond rotDiff = generalHelpfulTools::getQuaternionFromRPY(rotationX, rotationY, 0);
    return rotDiff;
}

Eigen::VectorXd ekfClassDVL::innovationStateDiff(Eigen::VectorXd z, Eigen::MatrixXd H,
                                                 Eigen::VectorXd currentStateBeforeUpdate) {//xyz vxvyvz rpy rvel pvel yvel
    Eigen::VectorXd innovation;
    innovation = z - H * currentStateBeforeUpdate;//also y
    innovation(6) = generalHelpfulTools::angleDiff(z(6), H(6, 6) * currentStateBeforeUpdate(6));
    innovation(7) = generalHelpfulTools::angleDiff(z(7), H(7, 7) * currentStateBeforeUpdate(7));
    innovation(8) = generalHelpfulTools::angleDiff(z(8), H(8, 8) * currentStateBeforeUpdate(8));

    return innovation;
}