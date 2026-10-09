// Created by jurobotics on 13.09.21.
// Derived from constructor-robotics/underwaterSLAM_2D_lowCost; see NOTICE.
// Local changes: three-axis IMU/DVL mounting parameters, gyro-bias subtraction
// in the IMU frame before rotation, invalid-DVL rejection, validity counters.
// This EKF provides the DVL/IMU odometry used by Ping-SLAM.
// gyroBiasX/Y/Z are subtracted from the measured angular velocity in rad/s.
// Use calibration appropriate to the recording; zero disables subtraction.
// The bias is defined in IMU coordinates and is removed before the mounting
// rotation. Values are dataset-specific and must not be copied between bags.
// Skipping invalid DVL updates avoids injecting a zero-velocity measurement;
// it does not guarantee accurate inertial propagation through long outages.

#include "ekfDVL.h"
#include "rclcpp/rclcpp.hpp"

// just for tricking compiler
#include "waterlinked_a50/msg/transducer_report_stamped.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "px4_msgs/msg/sensor_combined.hpp"


#include <thread>

static constexpr double CONSTANTS_ONE_G = 9.80665;

class RosClassEKF : public rclcpp::Node {
public:
    RosClassEKF() : Node("ekfNode"), currentEkf(rclcpp::Clock(RCL_ROS_TIME).now()) {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        this->declare_parameter("xPositionDVL", 0.0);
        this->declare_parameter("yPositionDVL", 0.0);
        this->declare_parameter("zPositionDVL", 0.0);
        this->declare_parameter("xPositionIMU", 0.0);
        this->declare_parameter("yPositionIMU", 0.0);
        this->declare_parameter("zPositionIMU", 0.0);
        this->declare_parameter("zPositionDepthSensor", 0.0);

        // IMU 三轴安装旋转 (默认 0 = 数据已在体系)
        this->declare_parameter("rollMountIMU", 0.0);
        this->declare_parameter("pitchMountIMU", 0.0);
        this->declare_parameter("yawMountIMU", 0.0);

        // [v3] 陀螺常值零偏 (rad/s, 定义在 IMU 自身坐标轴上, mount 旋转之前扣除)
        // 设 0 = 不补偿 (消融对照组)。甲鱼池长包实测: gyroBiasY = -0.01060
        this->declare_parameter("gyroBiasX", 0.0);
        this->declare_parameter("gyroBiasY", 0.0);
        this->declare_parameter("gyroBiasZ", 0.0);

        // [v2] DVL 三轴安装旋转 (默认 0; 老数据集: yawMountDVL 填原 yawRotationDVL 值)
        this->declare_parameter("rollMountDVL", 0.0);
        this->declare_parameter("pitchMountDVL", 0.0);
        this->declare_parameter("yawMountDVL", 0.0);

        this->firstMessage = true;

        // R_body_dvl = Rz(yaw) * Ry(pitch) * Rx(roll), 与 URDF rpy 约定一致
        this->rotationOfDVL =
                Eigen::AngleAxisd(this->get_parameter("yawMountDVL").as_double(),
                                  Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(this->get_parameter("pitchMountDVL").as_double(),
                                  Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(this->get_parameter("rollMountDVL").as_double(),
                                  Eigen::Vector3d::UnitX());

        this->positionIMU = Eigen::Vector3d(this->get_parameter("xPositionIMU").as_double(),
                                            this->get_parameter("yPositionIMU").as_double(),
                                            this->get_parameter("zPositionIMU").as_double());
        this->positionDVL = Eigen::Vector3d(this->get_parameter("xPositionDVL").as_double(),
                                            this->get_parameter("yPositionDVL").as_double(),
                                            this->get_parameter("zPositionDVL").as_double());

        // [v3] 读取陀螺零偏并在启动横幅里打出来, 便于确认 yaml 真的被读到
        this->gyroBias = Eigen::Vector3d(this->get_parameter("gyroBiasX").as_double(),
                                         this->get_parameter("gyroBiasY").as_double(),
                                         this->get_parameter("gyroBiasZ").as_double());
        RCLCPP_INFO(this->get_logger(),
                    "[ekfNode] gyro bias = (%+.5f, %+.5f, %+.5f) rad/s%s",
                    this->gyroBias.x(), this->gyroBias.y(), this->gyroBias.z(),
                    this->gyroBias.norm() < 1e-9 ? "   <<< 未补偿 (消融对照组) >>>" : "");

        // IMU 安装旋转写入 EKF (IMU系 -> 体系)
        this->currentEkf.setImuMountRotation(this->get_parameter("rollMountIMU").as_double(),
                                             this->get_parameter("pitchMountIMU").as_double(),
                                             this->get_parameter("yawMountIMU").as_double());


        this->subscriberIMU = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos,
                                                                               std::bind(&RosClassEKF::imuCallback,
                                                                                         this, std::placeholders::_1));

        this->subscriberDVL = this->create_subscription<waterlinked_a50::msg::TransducerReportStamped>(
                "/velocity_estimate", qos, std::bind(&RosClassEKF::DVLCallbackDVL, this, std::placeholders::_1));

        this->subscriberDepthSensorBaroSensorTube = this->create_subscription<sensor_msgs::msg::FluidPressure>(
                "/pressure", qos,
                std::bind(
                        &RosClassEKF::depthSensorBaroSensorTubeCallback,
                        this,
                        std::placeholders::_1));

        this->publisherPoseEkf = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "publisherPoseEkf", qos);
        this->publisherTwistEkf = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                "publisherTwistEkf", qos);


    }

private:
    ekfClassDVL currentEkf;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberIMU;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriberPX4IMU;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriberDepthSensorBaroSensorTube;
    rclcpp::Subscription<waterlinked_a50::msg::TransducerReportStamped>::SharedPtr subscriberDVL;


    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherPoseEkf;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisherTwistEkf;
    std::mutex updateEKFMutex;
    Eigen::Quaterniond rotationOfDVL;
    Eigen::Vector3d positionIMU, positionDVL;
    Eigen::Vector3d gyroBias;              // [v3] 陀螺常值零偏 (IMU 自身轴)

    double pressureWhenStarted;
    bool firstMessage;

    // [v4] DVL 掉点统计 (仅诊断用, 不参与滤波)
    long dvlValidCount   = 0;
    long dvlInvalidCount = 0;

    void imuCallbackHelper(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 原始 IMU 系数据直通 EKF; mount 旋转与 roll/pitch 提取在 ekfClassDVL 内完成
        // currentRotation 形参在 EKF 内未被使用, 传单位四元数即可
        Eigen::Quaterniond tmpRot = Eigen::Quaterniond::Identity();

        // predictionImu 只使用线加速度, 不涉及角速度, 无需扣偏置
        currentEkf.predictionImu(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                 msg->linear_acceleration.z,
                                 tmpRot, this->positionIMU,
                                 msg->header.stamp);

        // [v3] 扣除陀螺常值零偏后再送进 EKF。
        //   减的是 IMU 原始轴上的偏置, mount 旋转在 ekfClassDVL 内部做, 顺序正确。
        currentEkf.updateIMU(msg->linear_acceleration.x, msg->linear_acceleration.y,
                             msg->linear_acceleration.z,
                             msg->angular_velocity.x - this->gyroBias.x(),
                             msg->angular_velocity.y - this->gyroBias.y(),
                             msg->angular_velocity.z - this->gyroBias.z(),
                             tmpRot, msg->header.stamp);

        pose currentStateEkf = currentEkf.getState();
        geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
        poseMsg.header.frame_id = "map_ned";
        poseMsg.pose.pose.position.x = currentStateEkf.position.x();
        poseMsg.pose.pose.position.y = currentStateEkf.position.y();
        poseMsg.pose.pose.position.z = currentStateEkf.position.z();
        Eigen::Quaterniond rotDiff = currentEkf.getRotationVector();
        poseMsg.pose.pose.orientation.x = rotDiff.x();
        poseMsg.pose.pose.orientation.y = rotDiff.y();
        poseMsg.pose.pose.orientation.z = rotDiff.z();
        poseMsg.pose.pose.orientation.w = rotDiff.w();
        poseMsg.header.stamp = msg->header.stamp;
        this->publisherPoseEkf->publish(poseMsg);
        geometry_msgs::msg::TwistWithCovarianceStamped twistMsg;
        twistMsg.header.frame_id = "map_ned";
        twistMsg.twist.twist.linear.x = currentStateEkf.velocity.x();
        twistMsg.twist.twist.linear.y = currentStateEkf.velocity.y();
        twistMsg.twist.twist.linear.z = currentStateEkf.velocity.z();
        twistMsg.twist.twist.angular.x = currentStateEkf.angleVelocity.x();
        twistMsg.twist.twist.angular.y = currentStateEkf.angleVelocity.y();
        twistMsg.twist.twist.angular.z = currentStateEkf.angleVelocity.z();
        twistMsg.header.stamp = msg->header.stamp;
        this->publisherTwistEkf->publish(twistMsg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->updateEKFMutex.lock();
        this->imuCallbackHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void DVLCallbackDVLHelper(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        if (!msg->report.velocity_valid || msg->report.status != 0) {
            // [v4] 原实现在这里用 updateDVL(0,0,0,...) 更新, 等于告诉 EKF "艇静止"。
            //   甲鱼池实测 DVL 无效率 34.7%, 掉点段中位仅 0.35s (P90 0.58s) 属瞬时失锁,
            //   按 0 更新会把全程航程从 240m 压到 153m —— 长边 54m 变 34m。
            //   正确做法: 不更新, 让 EKF 靠 IMU 预测推进过这零点几秒。
            this->dvlInvalidCount++;
            return;
        }

        this->dvlValidCount++;
        this->currentEkf.updateDVL(msg->report.vx, msg->report.vy, msg->report.vz,
                                   this->rotationOfDVL,
                                   this->positionDVL,
                                   rclcpp::Time(msg->timestamp));

        // 每 2000 条 (≈4min @8.6Hz) 报一次有效率, 便于对照离线统计值 65.3%
        long total = this->dvlValidCount + this->dvlInvalidCount;
        if (total % 2000 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[ekfNode] DVL 有效率 %.1f%%  (有效 %ld / 无效 %ld)",
                        100.0 * (double) this->dvlValidCount / (double) total,
                        this->dvlValidCount, this->dvlInvalidCount);
        }
        return;
    }

    void DVLCallbackDVL(const waterlinked_a50::msg::TransducerReportStamped::SharedPtr msg) {
        this->updateEKFMutex.lock();
        this->DVLCallbackDVLHelper(msg);
        this->updateEKFMutex.unlock();
    }

    void depthSensorBaroSensorTubeCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {

        if (this->firstMessage) {
            this->pressureWhenStarted = msg->fluid_pressure;
            this->firstMessage = false;
            return;
        }


        double currentHeight =
                ((msg->fluid_pressure - this->pressureWhenStarted) * 0.01f) / (CONSTANTS_ONE_G * 1000.0f);

        this->updateEKFMutex.lock();
        this->depthSensorHelper(currentHeight, msg->header.stamp);
        this->updateEKFMutex.unlock();
    }



    void depthSensorHelper(double height, rclcpp::Time rosTime) {
        this->currentEkf.updateHeight(height, rosTime);
    }


    Eigen::Vector3d getRollPitchYaw(Eigen::Quaterniond quat) {
        tf2::Quaternion tmp(quat.x(), quat.y(), quat.z(), quat.w());
        tf2::Matrix3x3 m(tmp);
        double r, p, y;
        m.getRPY(r, p, y);
        Eigen::Vector3d returnVector(r, p, y);
        return returnVector;
    }

    Eigen::Quaterniond getQuaternionFromRPY(double roll, double pitch, double yaw) {
        tf2::Quaternion qtf2;
        qtf2.setRPY(roll, pitch, yaw);
        Eigen::Quaterniond q;
        q.x() = qtf2.x();
        q.y() = qtf2.y();
        q.z() = qtf2.z();
        q.w() = qtf2.w();
        return q;
    };

};

int main(int argc, char **argv) {


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosClassEKF>());


    rclcpp::shutdown();

    return (0);
}
