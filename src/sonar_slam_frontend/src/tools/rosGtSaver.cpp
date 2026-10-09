//
// GT 保存节点：直接订阅 /rigid_bodies，把完整 mocap 数据保存为 TUM 格式
// 独立订阅动捕消息；只保存节点运行期间收到且通过有效性检查的位姿。
// z 强制为 0，四元数只保留偏航角，与 2D SLAM 保持一致
// NaN/Inf 数据自动过滤
//

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <fstream>
#include <iomanip>
#include <mutex>
#include <cmath>
#include <Eigen/Geometry>

class rosGtSaver : public rclcpp::Node {
public:
    rosGtSaver() : Node("rosGtSaver") {

        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.durability_volatile();

        this->subscriberGT =
            this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
                "/rigid_bodies", qos,
                std::bind(&rosGtSaver::groundTruthCallback,
                          this, std::placeholders::_1));

        this->saveService = this->create_service<std_srvs::srv::Trigger>(
            "save_gt",
            std::bind(&rosGtSaver::saveGtCallback,
                      this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "GT Saver node started. Call /save_gt to save.");
    }

private:
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscriberGT;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveService;
    std::mutex gtMutex;

    struct GtPose {
        double timestamp;
        double x, y;
        double qz, qw;
    };
    std::vector<GtPose> gtPoses;

    void groundTruthCallback(
            const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
        if (msg->rigidbodies.empty()) return;

        // 过滤 NaN/Inf
        double px = msg->rigidbodies[0].pose.position.x;
        double py = msg->rigidbodies[0].pose.position.y;
        if (std::isnan(px) || std::isinf(px) ||
            std::isnan(py) || std::isinf(py)) return;

        Eigen::Quaterniond q(
            msg->rigidbodies[0].pose.orientation.w,
            msg->rigidbodies[0].pose.orientation.x,
            msg->rigidbodies[0].pose.orientation.y,
            msg->rigidbodies[0].pose.orientation.z);
        if (q.norm() < 1e-6) return;
        if (std::isnan(q.x()) || std::isnan(q.y()) ||
            std::isnan(q.z()) || std::isnan(q.w())) return;
        q.normalize();

        // 从完整四元数提取偏航角，重建只有偏航角的 2D 四元数
        double yaw = std::atan2(
            2.0 * (q.w() * q.z() + q.x() * q.y()),
            1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        GtPose pose;
        pose.timestamp = rclcpp::Time(msg->header.stamp).seconds();
        pose.x   = px;
        pose.y   = py;
        pose.qz  = std::sin(yaw / 2.0);
        pose.qw  = std::cos(yaw / 2.0);

        std::lock_guard<std::mutex> lock(this->gtMutex);
        this->gtPoses.push_back(pose);
    }

    void saveGtCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(this->gtMutex);

        if (this->gtPoses.empty()) {
            res->success = false;
            res->message = "No GT data received yet.";
            RCLCPP_WARN(this->get_logger(), "No GT data to save!");
            return;
        }

        std::string filename = "gtTrajectory_full_DATA02.txt";
        std::ofstream out(filename);
        out << std::fixed << std::setprecision(6);
        for (auto &p : this->gtPoses) {
            out << p.timestamp << " "
                << p.x   << " " << p.y   << " " << "0.000000" << " "
                << "0.000000" << " " << "0.000000" << " "
                << p.qz  << " " << p.qw  << "\n";
        }
        out.close();

        RCLCPP_INFO(this->get_logger(), "Saved %zu GT poses to %s",
                    this->gtPoses.size(), filename.c_str());
        res->success = true;
        res->message = "Saved " + filename + " with " +
                       std::to_string(this->gtPoses.size()) + " poses.";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosGtSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
