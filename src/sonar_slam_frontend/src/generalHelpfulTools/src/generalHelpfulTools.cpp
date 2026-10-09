//
// Created by jurobotics on 15.09.21.
//

#include "generalHelpfulTools.h"

Eigen::Vector3d generalHelpfulTools::getRollPitchYaw(Eigen::Quaterniond quat) {
    tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
    tf2::Matrix3x3 m(tmp);
    double r, p, y;
    m.getRPY(r, p, y);
    Eigen::Vector3d returnVector(r, p, y);
    return returnVector;
}

Eigen::Matrix4d generalHelpfulTools::getTransformationMatrixFromRPY(double roll, double pitch, double yaw) {
    Eigen::Quaterniond rotationAsQuaternion = generalHelpfulTools::getQuaternionFromRPY(roll, pitch, yaw);
    Eigen::Matrix4d returnMatrix = Eigen::Matrix4d::Identity();
    returnMatrix.block<3, 3>(0, 0) = rotationAsQuaternion.toRotationMatrix();
    return returnMatrix;
}

Eigen::Quaterniond generalHelpfulTools::getQuaternionFromRPY(double roll, double pitch, double yaw) {
    tf2::Quaternion qtf2;
    qtf2.setRPY(roll, pitch, yaw);
    Eigen::Quaterniond q;
    q.x() = qtf2.x();
    q.y() = qtf2.y();
    q.z() = qtf2.z();
    q.w() = qtf2.w();
    return q;
}

double generalHelpfulTools::angleDiff(double first, double second) {//first-second
    return atan2(sin(first - second), cos(first - second));
}


Eigen::Matrix4d generalHelpfulTools::interpolationTwo4DTransformations(Eigen::Matrix4d &transformation1,
                                                                       Eigen::Matrix4d &transformation2, double &t) {
    //computes the transofrmation matrix at time point t between 2 and 1
    if (t < 0 || t > 1) {
        std::cout << "t value not between 0 and 1: " << t << std::endl;
        exit(-1);
    }
    Eigen::Vector3d translation1 = transformation1.block<3, 1>(0, 3);
    Eigen::Vector3d translation2 = transformation2.block<3, 1>(0, 3);
    Eigen::Quaterniond rot1(transformation1.block<3, 3>(0, 0));
    Eigen::Quaterniond rot2(transformation2.block<3, 3>(0, 0));

    Eigen::Quaterniond resultingRot = rot1.slerp(t, rot2);
    Eigen::Vector3d resultingTranslation = translation1 * t + translation2 * (1.0 - t);

    Eigen::Matrix4d resultingTransformation = Eigen::Matrix4d::Identity();
    resultingTransformation.block<3, 3>(0, 0) = resultingRot.toRotationMatrix();
    resultingTransformation.block<3, 1>(0, 3) = resultingTranslation;

    return resultingTransformation;


}


Eigen::Matrix4d
generalHelpfulTools::getTransformationMatrix(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 1>(0, 3) = translation;
    transformation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    return transformation;
}

double generalHelpfulTools::weighted_mean(const std::vector<double> &data) {
    double mean = 0.0;

    for (int i = 0; i < data.size(); i++) {
        mean += data[i];
    }
    return mean / double(data.size());
}

double generalHelpfulTools::normalizeAngle(double inputAngle){

    while(inputAngle<0){
        inputAngle = inputAngle+M_PI*2;
    }
    inputAngle = inputAngle+M_PI*2;

    return std::fmod(inputAngle,M_PI*2);

}

