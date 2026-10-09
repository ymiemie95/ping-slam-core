// Ping360 beam-to-pseudo-scan front end for Ping-SLAM.
// Project integration code; upstream geometry helpers and the pose-cache
// structure are identified below and in this package's NOTICE.
//
// Beam angles are converted from 400 sonar units per revolution to radians.
// The static base_link -> sonar_link transform carries the mounting rotation.
// EKF poses compensate motion before samples are placed in a LaserScan.
//
// range_extraction_method selects max or max_bgsub. The latter subtracts a
// cross-beam median background; optional residual_zscore uses MAD scaling.
// Filters, minimum-valid-bin gating and optional segment densification are
// controlled by parameters. Densification supports distance and angle modes.
// Range limits and mounting/bias parameters must match the dataset.
// Dataset-specific experiments are recorded separately; they are not universal
// guarantees about clutter rejection or reconstruction accuracy.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <deque>
#include <vector>
#include <mutex>
#include <limits>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// 工具函数 (generalHelpfulTools 源自 underwaterSLAM_2D_lowCost, 见本包 NOTICE)
#include "generalHelpfulTools.h"

// EKF 位姿缓存条目 (原 slamToolsRos.h 里的同名结构体, 结构来源见本包 NOTICE；此处只保留位姿和时间戳)
struct transformationStamped {
    Eigen::Matrix4d transformation;
    double timeStamp;
};

// 复用声呐消息
#include "ping360_sonar_msgs/msg/sonar_echo.hpp"

// ============================================================
class SonarToLaserscan : public rclcpp::Node {
public:
    SonarToLaserscan() : Node("sonarToLaserscan") {

        // ---- 距离提取 ----
        this->declare_parameter("range_extraction_method", std::string("max_bgsub"));
        this->declare_parameter("intensity_threshold", 65.0);   // 仅 max 使用
        this->declare_parameter("bg_window_beams",    100);     // step=4 → 100束 = 一整圈
        this->declare_parameter("bg_min_fill_beams",  25);      // 冷启动保护
        this->declare_parameter("residual_threshold", 20.0);    // max_bgsub 残差判决阈值
        // 2026-09-17 CFAR 式归一化残差 (仅 max_bgsub): 残差除以该格在方位窗内的稳健离散度
        //   sigma_k = 1.4826*MAD_k + residual_noise_floor, 取 z 最大的格, 要求 z >= residual_zscore
        //   且残差 >= residual_threshold; residual_zscore = 0 关闭 (= 原 max_bgsub)。
        this->declare_parameter("residual_zscore",     0.0);
        this->declare_parameter("residual_noise_floor", 3.0);

        // ---- LaserScan 栅格与距离门 ----
        this->declare_parameter("angle_increment",     2.0 * M_PI / 100.0);
        this->declare_parameter("range_min",           12.0);
        this->declare_parameter("range_max",           20.0);

        // ---- 切帧策略 ----
        this->declare_parameter("frame_accumulate_angle", 3.0 * M_PI / 2.0);
        this->declare_parameter("publish_interval_angle", M_PI / 2.0);
        this->declare_parameter("min_beams_per_scan",  40);
        // ★真正起作用的闸门：有效栅格数不足则整帧丢弃
        this->declare_parameter("min_valid_bins_per_scan", 12);

        // ---- 孤立点滤波 ----
        this->declare_parameter("enable_scan_outlier_filter", true);
        this->declare_parameter("scan_outlier_distance",      1.0);
        // 容差随距离线性放宽: tol = distance + range_ratio * r
        this->declare_parameter("scan_outlier_range_ratio",   0.0);
        this->declare_parameter("scan_outlier_half_window",   2);
        this->declare_parameter("scan_outlier_min_neighbors", 2);

        // ---- 连续段滤波（只用"地图是连续曲线"先验, 无形状假设）----
        this->declare_parameter("enable_segment_filter",  true);
        this->declare_parameter("segment_max_gap_bins",   2);     // 段内允许的bin空洞
        this->declare_parameter("segment_jump_per_bin",   1.2);   // 相邻点径向跳变 m/bin
        this->declare_parameter("segment_min_points",     4);     // 成段最少点数

        // ---- 结构化加密 (段内弦线插值, 观测增强) ----
        // validBins 闸门在加密【之前】用原始点数判定 —— 插值点不参与闸门。
        this->declare_parameter("enable_segment_densify", false);
        this->declare_parameter("densify_step",           0.15);  // 弦线采样间隔 m
        this->declare_parameter("densify_output_bins",    400);   // 输出角分辨率(0.9°)
        // ★[D] 弦长上限★ 超过此长度的弦不插值(只保留真实端点), 防噪声桥/远距长弦
        this->declare_parameter("densify_max_chord",      1.5);
        // ★[E] 2026-09-23 加密采样律 (导师意见: 输出是等角 LaserScan, 应按角度加密)
        //   "distance": 沿弦线按 densify_step 等距采样, 再落细 bin (原方案)
        //   "angle"   : 对弦线两端之间的每一个细 bin, 取该 bin 中心方位射线与弦线的
        //               交点作为该 bin 的距离 —— 每个输出 bin 恰好一个样本, 不空不重。
        //               densify_step 在此模式下不参与采样 (仅保留 maxChord 钳位)。
        this->declare_parameter("densify_mode",           std::string("distance"));

        // ---- TF ----
        this->declare_parameter("laser_frame",         std::string("sonar_link"));
        this->declare_parameter("base_frame",          std::string("base_link"));
        this->declare_parameter("odom_frame",          std::string("odom"));
        this->declare_parameter("sonar_x",   0.0);
        this->declare_parameter("sonar_y",   0.0);
        this->declare_parameter("sonar_z",   0.0);
        this->declare_parameter("sonar_yaw", M_PI);

        // ---- 读取参数 ----
        rangeExtractionMethod_ = this->get_parameter("range_extraction_method").as_string();
        intensityThreshold_    = this->get_parameter("intensity_threshold").as_double();
        bgWindowBeams_         = (int)this->get_parameter("bg_window_beams").as_int();
        bgMinFillBeams_        = (int)this->get_parameter("bg_min_fill_beams").as_int();
        residualThreshold_     = this->get_parameter("residual_threshold").as_double();
        residualZscore_        = this->get_parameter("residual_zscore").as_double();
        residualNoiseFloor_    = this->get_parameter("residual_noise_floor").as_double();

        angleIncrement_        = this->get_parameter("angle_increment").as_double();
        rangeMin_              = this->get_parameter("range_min").as_double();
        rangeMax_              = this->get_parameter("range_max").as_double();

        frameAccumulateAngle_  = this->get_parameter("frame_accumulate_angle").as_double();
        publishIntervalAngle_  = this->get_parameter("publish_interval_angle").as_double();
        minBeamsPerScan_       = (int)this->get_parameter("min_beams_per_scan").as_int();
        minValidBinsPerScan_   = (int)this->get_parameter("min_valid_bins_per_scan").as_int();

        enableOutlierFilter_   = this->get_parameter("enable_scan_outlier_filter").as_bool();
        outlierDistance_       = this->get_parameter("scan_outlier_distance").as_double();
        outlierRangeRatio_     = this->get_parameter("scan_outlier_range_ratio").as_double();
        outlierHalfWindow_     = (int)this->get_parameter("scan_outlier_half_window").as_int();
        outlierMinNeighbors_   = (int)this->get_parameter("scan_outlier_min_neighbors").as_int();

        enableSegmentFilter_ = this->get_parameter("enable_segment_filter").as_bool();
        segMaxGapBins_       = (int)this->get_parameter("segment_max_gap_bins").as_int();
        segJumpPerBin_       = this->get_parameter("segment_jump_per_bin").as_double();
        segMinPoints_        = (int)this->get_parameter("segment_min_points").as_int();
        enableSegmentDensify_ = this->get_parameter("enable_segment_densify").as_bool();
        densifyStep_          = this->get_parameter("densify_step").as_double();
        densifyOutputBins_    = (int)this->get_parameter("densify_output_bins").as_int();
        densifyMaxChord_      = this->get_parameter("densify_max_chord").as_double();
        densifyMode_          = this->get_parameter("densify_mode").as_string();
        if (densifyMode_ != "distance" && densifyMode_ != "angle") {
            RCLCPP_WARN(this->get_logger(),
                "[sonarToLaserscan] densify_mode='%s' 未知, 回退 distance", densifyMode_.c_str());
            densifyMode_ = "distance";
        }
        if (densifyStep_ < 0.02) densifyStep_ = 0.02;
        if (densifyOutputBins_ < N_) densifyOutputBins_ = N_;
        if (densifyMaxChord_ < densifyStep_) densifyMaxChord_ = densifyStep_;
        if (enableSegmentDensify_ && !enableSegmentFilter_) {
            RCLCPP_WARN(this->get_logger(),
                "[sonarToLaserscan] enable_segment_densify 需要 enable_segment_filter=true "
                "(插值定义域是段滤波输出), 已自动关闭加密");
            enableSegmentDensify_ = false;
        }

        laserFrame_ = this->get_parameter("laser_frame").as_string();
        baseFrame_  = this->get_parameter("base_frame").as_string();
        odomFrame_  = this->get_parameter("odom_frame").as_string();
        sonarX_     = this->get_parameter("sonar_x").as_double();
        sonarY_     = this->get_parameter("sonar_y").as_double();
        sonarZ_     = this->get_parameter("sonar_z").as_double();
        sonarYaw_   = this->get_parameter("sonar_yaw").as_double();

        if (bgWindowBeams_ < 10) bgWindowBeams_ = 10;
        if (bgMinFillBeams_ > bgWindowBeams_) bgMinFillBeams_ = bgWindowBeams_;

        // 防御：发布步进不应超过窗口宽度，否则会漏波束
        if (publishIntervalAngle_ > frameAccumulateAngle_) {
            RCLCPP_WARN(this->get_logger(),
                "[sonarToLaserscan] publish_interval_angle(%.3f) > frame_accumulate_angle(%.3f)，"
                "已钳到窗口宽度（overlap=0）", publishIntervalAngle_, frameAccumulateAngle_);
            publishIntervalAngle_ = frameAccumulateAngle_;
        }

        // N = round(2π / angle_increment)，并反算精确步进角消浮点零头
        N_ = (int)std::round(2.0 * M_PI / angleIncrement_);
        if (N_ < 8) N_ = 8;
        angleIncrement_ = 2.0 * M_PI / N_;

        if (minValidBinsPerScan_ > N_) minValidBinsPerScan_ = N_;

        // 预计算 base→sonar 外参 T_BS 及其逆
        T_BS_ = generalHelpfulTools::getTransformationMatrixFromRPY(0.0, 0.0, sonarYaw_);
        T_BS_(0, 3) = sonarX_;
        T_BS_(1, 3) = sonarY_;
        T_BS_(2, 3) = sonarZ_;
        T_BS_inv_ = T_BS_.inverse();

        // ---- 静态 TF: base_link → sonar_link ----
        staticTfBroadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishStaticTF();

        // ---- 动态 TF broadcaster: odom → base_link ----
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ---- Publishers & Subscribers ----
        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.durability_volatile();

        scanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        cbGrpEkf_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbGrpSonar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto ekfOpt = rclcpp::SubscriptionOptions();
        ekfOpt.callback_group = cbGrpEkf_;
        auto sonarOpt = rclcpp::SubscriptionOptions();
        sonarOpt.callback_group = cbGrpSonar_;

        ekfSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "publisherPoseEkf", qos,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                ekfCallback(msg);
            }, ekfOpt);

        sonarSub_ = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
            "sonar/intensity", qos,
            [this](const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
                sonarCallback(msg);
            }, sonarOpt);

        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] N=%d  angle_inc=%.5f  range=[%.2f,%.2f]  "
            "window=%.4f step=%.4f (overlap %.0f%%)  method=%s",
            N_, angleIncrement_, rangeMin_, rangeMax_,
            frameAccumulateAngle_, publishIntervalAngle_,
            100.0 * (1.0 - publishIntervalAngle_ / frameAccumulateAngle_),
            rangeExtractionMethod_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] bgWin=%d residThr=%.1f intThr=%.1f zscore=%.1f floor=%.1f | "
            "minBeams=%d  ★minValidBins=%d★ | outlierFilter=%d "
            "(tol=%.2f+%.3f*r, halfWin=%d, minNb=%d) | laser=%s odom=%s",
            bgWindowBeams_, residualThreshold_, intensityThreshold_, residualZscore_, residualNoiseFloor_,
            minBeamsPerScan_, minValidBinsPerScan_,
            (int)enableOutlierFilter_, outlierDistance_, outlierRangeRatio_,
            outlierHalfWindow_, outlierMinNeighbors_,
            laserFrame_.c_str(), odomFrame_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] segmentFilter=%d (maxGap=%d bins, jump=%.2f m/bin, "
            "minPoints=%d)",
            (int)enableSegmentFilter_, segMaxGapBins_, segJumpPerBin_, segMinPoints_);
        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] segmentDensify=%d (mode=%s, step=%.2f m, ★maxChord=%.2f m★, outputBins=%d)",
            (int)enableSegmentDensify_, densifyMode_.c_str(), densifyStep_, densifyMaxChord_, densifyOutputBins_);
    }

private:
    // ----------------------------------------------------------------
    // 静态 TF: base_link → sonar_link
    // ----------------------------------------------------------------
    void publishStaticTF() {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = this->now();
        ts.header.frame_id = baseFrame_;
        ts.child_frame_id  = laserFrame_;

        ts.transform.translation.x = sonarX_;
        ts.transform.translation.y = sonarY_;
        ts.transform.translation.z = sonarZ_;

        double halfYaw = sonarYaw_ / 2.0;
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = std::sin(halfYaw);
        ts.transform.rotation.w = std::cos(halfYaw);

        staticTfBroadcaster_->sendTransform(ts);
        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] Static TF %s→%s  yaw=%.4f rad",
            baseFrame_.c_str(), laserFrame_.c_str(), sonarYaw_);
    }

    // ----------------------------------------------------------------
    // EKF 回调：缓冲位姿 + 发布 odom → base_link 动态 TF
    // ----------------------------------------------------------------
    void ekfCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ekfMutex_);
        double ts = rclcpp::Time(msg->header.stamp).seconds();

        Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                              msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z);
        Eigen::Vector3d p(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);

        int i = (int)ekfBuffer_.size();
        while (i > 0 && ekfBuffer_[i-1].timeStamp > ts) i--;

        transformationStamped tStamped;
        tStamped.timeStamp      = ts;
        tStamped.transformation = generalHelpfulTools::getTransformationMatrix(p, q);

        ekfBuffer_.insert(ekfBuffer_.begin() + i, tStamped);
        while ((int)ekfBuffer_.size() > 5000)
            ekfBuffer_.pop_front();

        geometry_msgs::msg::TransformStamped tfMsg;
        tfMsg.header.stamp    = msg->header.stamp;
        tfMsg.header.frame_id = odomFrame_;
        tfMsg.child_frame_id  = baseFrame_;
        tfMsg.transform.translation.x = p.x();
        tfMsg.transform.translation.y = p.y();
        tfMsg.transform.translation.z = p.z();
        tfMsg.transform.rotation.x = q.x();
        tfMsg.transform.rotation.y = q.y();
        tfMsg.transform.rotation.z = q.z();
        tfMsg.transform.rotation.w = q.w();
        tfBroadcaster_->sendTransform(tfMsg);
    }

    // ----------------------------------------------------------------
    // 等待 EKF 缓冲追上目标时间戳
    // ----------------------------------------------------------------
    bool waitForEKF(double targetTs) {
        const int maxWaitMs = 2000;
        const int stepMs    = 10;
        for (int w = 0; w < maxWaitMs / stepMs; ++w) {
            {
                std::lock_guard<std::mutex> lock(ekfMutex_);
                if (!ekfBuffer_.empty() && ekfBuffer_.back().timeStamp >= targetTs)
                    return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(stepMs));
        }
        RCLCPP_WARN(this->get_logger(),
            "[sonarToLaserscan] waitForEKF timeout ts=%.3f", targetTs);
        return false;
    }

    // ----------------------------------------------------------------
    // 内部数据结构
    // ----------------------------------------------------------------
    struct BeamEntry {
        double angle;
        double range;
        double ts;
        double cumAngle;               // 单调累计转角，用于滑窗裁剪
        Eigen::Matrix4d T_abs;
        std::vector<double> intensities;
        double extractedRange = -1.0;  // max_bgsub 在入队时一次性算好并缓存
                                       // （背景窗是"截至该束"的最近一圈，严格因果；
                                       //   重叠帧重放时不再重算，省 CPU 也保证一致）
    };

    // ----------------------------------------------------------------
    // 声呐回调：取位姿 → 背景窗更新 → 滑窗触发
    // ----------------------------------------------------------------
    void sonarCallback(const ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
        double ts = rclcpp::Time(msg->header.stamp).seconds();
        double beamAngle = msg->angle / 400.0 * 2.0 * M_PI;

        if (!waitForEKF(ts)) return;
        // [Timing] 计时从 EKF 等待之后开始: waitForEKF 里的 sleep 是等数据, 不是计算
        const auto tBeam0 = std::chrono::steady_clock::now();

        // 后向线性插值取该时刻绝对位姿
        Eigen::Matrix4d T_abs;
        {
            std::lock_guard<std::mutex> lock(ekfMutex_);
            if (ekfBuffer_.size() < 2) return;

            int idx = (int)ekfBuffer_.size() - 1;
            while (idx > 0 && ekfBuffer_[idx].timeStamp > ts) idx--;
            if (idx + 1 < (int)ekfBuffer_.size()) {
                double t0 = ekfBuffer_[idx].timeStamp;
                double t1 = ekfBuffer_[idx+1].timeStamp;
                double dt = t1 - t0;
                double alpha = (dt > 1e-9) ? (ts - t0) / dt : 0.0;
                alpha = std::max(0.0, std::min(1.0, alpha));
                T_abs = generalHelpfulTools::interpolationTwo4DTransformations(
                    ekfBuffer_[idx].transformation,
                    ekfBuffer_[idx+1].transformation,
                    alpha);
            } else {
                T_abs = ekfBuffer_[idx].transformation;
            }
        }

        // 单调累计总转角（滑窗用）
        if (!beamBuffer_.empty()) {
            double diff = std::abs(generalHelpfulTools::angleDiff(beamAngle, prevBeamAngle_));
            cumAngleTotal_ += diff;
        }
        prevBeamAngle_ = beamAngle;

        // 组装波束
        BeamEntry entry;
        entry.angle    = beamAngle;
        entry.range    = (double)msg->range;
        entry.ts       = ts;
        entry.cumAngle = cumAngleTotal_;
        entry.T_abs    = T_abs;
        entry.intensities.reserve(msg->intensities.size());
        for (auto v : msg->intensities)
            entry.intensities.push_back((double)v);

        // max_bgsub：先更新背景窗，再就地判决并缓存结果
        //   （bgWindow_ 只在 sonar 回调线程访问，cbGrpSonar_ 为 MutuallyExclusive，免锁）
        if (rangeExtractionMethod_ == "max_bgsub") {
            // 量程/采样数变化（换驱动配置）时背景剖面失效，整窗清空
            if (!bgWindow_.empty() &&
                (bgWindow_.back().size() != entry.intensities.size() ||
                 std::abs(bgWindowRange_ - entry.range) > 1e-6)) {
                bgWindow_.clear();
                RCLCPP_WARN(this->get_logger(),
                    "[sonarToLaserscan] 声呐量程/采样数变化，背景窗已重置");
            }
            bgWindowRange_ = entry.range;
            bgWindow_.push_back(entry.intensities);
            while ((int)bgWindow_.size() > bgWindowBeams_)
                bgWindow_.pop_front();

            entry.extractedRange = extractRangeBgsub(entry);
        }

        beamBuffer_.push_back(entry);

        // 丢弃滑窗外旧波束
        double windowStart = cumAngleTotal_ - frameAccumulateAngle_;
        while (!beamBuffer_.empty() && beamBuffer_.front().cumAngle < windowStart)
            beamBuffer_.pop_front();

        // [Timing] 每束波束的处理时间累计到下一次发布 (不含 buildAndPublishLaserScan 本身)
        beamMsAccum_ += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tBeam0).count();
        beamsSinceScan_++;

        // 触发发布
        if (cumAngleTotal_ >= frameAccumulateAngle_ &&
            (cumAngleTotal_ - lastPublishCumAngle_) >= publishIntervalAngle_ &&
            (int)beamBuffer_.size() >= minBeamsPerScan_) {
            const auto tBuild0 = std::chrono::steady_clock::now();   // [Timing]
            buildAndPublishLaserScan();
            const double buildMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - tBuild0).count();
            RCLCPP_INFO(this->get_logger(),
                "[Timing][frontend] scan=%ld build_ms=%.2f beams_ms=%.2f n_beams=%ld",
                (long)publishedCount_, buildMs, beamMsAccum_, beamsSinceScan_);
            beamMsAccum_ = 0.0;
            beamsSinceScan_ = 0;
            lastPublishCumAngle_ = cumAngleTotal_;
        }
    }

    // ----------------------------------------------------------------
    // 去畸变 + 落 bin + 滤波 + ★有效点闸门★ + 发布
    // ----------------------------------------------------------------
    void buildAndPublishLaserScan() {
        if (beamBuffer_.empty()) return;

        const Eigen::Matrix4d& T_ref     = beamBuffer_.back().T_abs;
        const Eigen::Matrix4d  T_ref_inv = T_ref.inverse();
        const double t_ref_ts   = beamBuffer_.back().ts;
        const double t_first_ts = beamBuffer_.front().ts;

        std::vector<float> ranges(N_, std::numeric_limits<float>::infinity());

        for (const auto& beam : beamBuffer_) {
            // 去畸变：base 相对运动共轭到 sonar 系
            Eigen::Matrix4d T_corr = T_BS_inv_ * T_ref_inv * beam.T_abs * T_BS_;

            Eigen::Matrix4d R_sonar =
                generalHelpfulTools::getTransformationMatrixFromRPY(0.0, 0.0, beam.angle);

            double r = extractRange(beam);
            if (r < rangeMin_ || r >= rangeMax_) continue;

            Eigen::Vector4d pt(r, 0.0, 0.0, 1.0);
            pt = T_corr * R_sonar * pt;

            double bearing = std::atan2(pt.y(), pt.x());
            double dist    = std::hypot(pt.x(), pt.y());

            int b = (int)((bearing + M_PI) / angleIncrement_);
            if (b < 0 || b >= N_) continue;
            if (dist > rangeMin_ && dist < rangeMax_ && dist < ranges[b])
                ranges[b] = (float)dist;
        }

        int binsBeforeFilter = 0;
        for (auto q : ranges) if (std::isfinite(q)) binsBeforeFilter++;

        // 孤立点滤波：有效bin与环形邻域中值偏差过大则剔除
        if (enableOutlierFilter_)
            filterScanOutliers(ranges);

        // 连续段滤波：只保留能连成 >=segment_min_points 点的连续段。
        // 只用连续性先验, 不做任何形状(直线/矩形)假设 —— 散斑因不连续被剔除。
        int segsFound = 0;
        std::vector<std::vector<int>> keptSegs;
        if (enableSegmentFilter_)
            segsFound = filterScanSegments(ranges, &keptSegs);

        int validBins = 0;
        for (auto q : ranges) if (std::isfinite(q)) validBins++;

        // ★★★ 闸门 ★★★
        // 有效点太少的帧不发布。这类帧在 Karto 里只会产生 response≈0、
        // var=(500,500) 的废边，把位姿图带偏。宁可少一帧，不要一帧废数据。
        if (validBins < minValidBinsPerScan_) {
            droppedCount_++;
            RCLCPP_WARN(this->get_logger(),
                "[sonarToLaserscan] 丢弃帧: validBins=%d < %d (滤波前 %d, beams=%zu, "
                "t=%.3f)  累计: 发布 %ld / 丢弃 %ld",
                validBins, minValidBinsPerScan_, binsBeforeFilter,
                beamBuffer_.size(), t_ref_ts, publishedCount_, droppedCount_);
            return;
        }

        // 结构化加密 (闸门之后 —— 闸门只认原始测量点)
        int densePts = 0;
        std::vector<float> denseRanges;
        if (enableSegmentDensify_) {
            denseRanges = densifySegments(ranges, keptSegs);
            for (auto q : denseRanges) if (std::isfinite(q)) densePts++;
        }
        const bool useDense = enableSegmentDensify_ && densePts >= validBins;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp    = rclcpp::Time(static_cast<int64_t>(t_ref_ts * 1e9), RCL_ROS_TIME);
        scan.header.frame_id = laserFrame_;

        scan.angle_min       = -(float)M_PI;
        scan.angle_max       = +(float)M_PI;
        scan.angle_increment = useDense ? (float)(2.0 * M_PI / densifyOutputBins_)
                                        : (float)angleIncrement_;
        scan.range_min       = (float)rangeMin_;
        scan.range_max       = (float)rangeMax_;
        scan.time_increment  = 0.0f;
        scan.scan_time       = (float)(t_ref_ts - t_first_ts);

        scan.ranges = useDense ? denseRanges : ranges;
        scan.intensities.clear();

        scanPub_->publish(scan);
        publishedCount_++;

        RCLCPP_INFO(this->get_logger(),
            "[sonarToLaserscan] Scan published: t=%.3f  scan_time=%.2f s  beams=%zu  "
            "validBins=%d/%d (滤波前 %d, 段数 %d, 加密后 %d)  累计: 发布 %ld / 丢弃 %ld",
            t_ref_ts, scan.scan_time, beamBuffer_.size(),
            validBins, N_, binsBeforeFilter, segsFound, densePts,
            publishedCount_, droppedCount_);
    }

    // ----------------------------------------------------------------
    // LaserScan 孤立点滤波（环形邻域一致性）
    //   容差 tol = scan_outlier_distance + scan_outlier_range_ratio * r
    // ----------------------------------------------------------------
    void filterScanOutliers(std::vector<float>& ranges) const {
        const int n = (int)ranges.size();
        std::vector<float> out = ranges;
        int removed = 0;

        for (int b = 0; b < n; ++b) {
            if (!std::isfinite(ranges[b])) continue;

            std::vector<float> nb;
            for (int d = -outlierHalfWindow_; d <= outlierHalfWindow_; ++d) {
                if (d == 0) continue;
                int j = ((b + d) % n + n) % n;   // 环形索引
                if (std::isfinite(ranges[j])) nb.push_back(ranges[j]);
            }
            if ((int)nb.size() < outlierMinNeighbors_) {
                out[b] = std::numeric_limits<float>::infinity();  // 邻域全空 = 孤立点
                removed++;
                continue;
            }
            std::nth_element(nb.begin(), nb.begin() + nb.size() / 2, nb.end());
            float med = nb[nb.size() / 2];
            float tol = (float)(outlierDistance_ + outlierRangeRatio_ * ranges[b]);
            if (std::abs(ranges[b] - med) > tol) {
                out[b] = std::numeric_limits<float>::infinity();
                removed++;
            }
        }
        if (removed > 0)
            RCLCPP_DEBUG(this->get_logger(),
                "[sonarToLaserscan] outlier filter removed %d bins", removed);
        ranges.swap(out);
    }

    // ----------------------------------------------------------------
    // 连续段滤波（仅连续性先验, 无形状假设）
    //   沿 bin 环扫描: 相邻有效点若 bin间隔<=segMaxGapBins_ 且
    //   径向跳变<=segJumpPerBin_×间隔, 归入同一段;
    //   只保留点数>=segMinPoints_ 的段, 其余置 inf。返回保留的段数。
    //   环形处理: 从最大空隙处切开展平, 避免把一条跨±π的墙劈成两段。
    //   复杂度 O(N), 每帧 <0.1ms。
    // ----------------------------------------------------------------
    int filterScanSegments(std::vector<float>& ranges,
                           std::vector<std::vector<int>>* keptSegs = nullptr) const {
        const int n = (int)ranges.size();
        std::vector<int> fin;
        for (int b = 0; b < n; ++b)
            if (std::isfinite(ranges[b])) fin.push_back(b);
        const int m = (int)fin.size();
        if (m == 0) return 0;
        if (m < segMinPoints_) {
            // 点太少不可能成段, 全部清掉, 交给 validBins 闸门丢帧
            for (int b : fin) ranges[b] = std::numeric_limits<float>::infinity();
            return 0;
        }

        // 找最大 bin 空隙, 从那里切开环 → 展平成线性序列
        int cut = 0, maxGap = -1;
        for (int k = 0; k < m; ++k) {
            int nxt = (k + 1) % m;
            int gap = (fin[nxt] - fin[k] + n) % n;
            if (k == m - 1) gap = (fin[0] + n - fin[m-1]) % n;   // 环回空隙
            if (gap > maxGap) { maxGap = gap; cut = k; }
        }
        std::vector<int> order(m);
        for (int k = 0; k < m; ++k) order[k] = fin[(cut + 1 + k) % m];

        // 分段
        std::vector<std::vector<int>> segs;
        segs.push_back({order[0]});
        for (int k = 1; k < m; ++k) {
            int a = order[k-1], b = order[k];
            int bingap = (b - a + n) % n;
            double rjump = std::fabs((double)ranges[b] - (double)ranges[a]);
            if (bingap <= segMaxGapBins_ && rjump <= segJumpPerBin_ * bingap) {
                segs.back().push_back(b);
            } else {
                segs.emplace_back(std::vector<int>{b});
            }
        }

        // 保留达标段
        std::vector<char> keep(n, 0);
        int nsegs = 0;
        for (const auto& s : segs) {
            if ((int)s.size() >= segMinPoints_) {
                for (int b : s) keep[b] = 1;
                nsegs++;
                if (keptSegs) keptSegs->push_back(s);   // 环序 bin 列表, 供加密用
            }
        }
        int removed = 0;
        for (int b : fin) {
            if (!keep[b]) {
                ranges[b] = std::numeric_limits<float>::infinity();
                removed++;
            }
        }
        if (removed > 0)
            RCLCPP_DEBUG(this->get_logger(),
                "[sonarToLaserscan] segment filter: %d segs kept, removed %d/%d pts",
                nsegs, removed, m);
        return nsegs;
    }

    // ----------------------------------------------------------------
    // 结构化加密: 段内弦线稠密采样 → 高角分辨率 ranges
    //   对每个保留段, 相邻两测量点连弦线, 以 densifyStep_ 间隔采样,
    //   采样点折算回 (方位, 距离) 落入 densifyOutputBins_ 个细 bin (同格取最近)。
    //   只在段内插值 —— 段间断裂可能是真实开口, 保持空白。
    //   ★[D] 弦长上限★: 弦长 > densifyMaxChord_ 的相邻点对不插值,
    //   只保留真实端点 —— 远距长弦与噪声端点连线不再被画成假墙。
    //   插值几何误差上界 = 边界曲率相对弦线的偏离, 光滑边界在 ~1m 弦长上
    //   <0.1m, 低于两倍栅格分辨率, 不引入虚假结构。
    // ----------------------------------------------------------------
    std::vector<float> densifySegments(const std::vector<float>& ranges,
                                       const std::vector<std::vector<int>>& segs) const {
        const int nd = densifyOutputBins_;
        const double dinc = 2.0 * M_PI / nd;
        std::vector<float> dense(nd, std::numeric_limits<float>::infinity());

        auto deposit = [&](double x, double y) {
            double d  = std::hypot(x, y);
            if (d < rangeMin_ || d > rangeMax_) return;
            int b = (int)((std::atan2(y, x) + M_PI) / dinc);
            if (b < 0) b = 0;
            if (b >= nd) b = nd - 1;
            if (d < dense[b]) dense[b] = (float)d;
        };

        for (const auto& s : segs) {
            for (size_t k = 0; k + 1 < s.size(); ++k) {
                int ba = s[k], bb = s[k + 1];
                double ta = ba * angleIncrement_ - M_PI;
                double tb = bb * angleIncrement_ - M_PI;
                double ax = ranges[ba] * std::cos(ta), ay = ranges[ba] * std::sin(ta);
                double bx = ranges[bb] * std::cos(tb), by = ranges[bb] * std::sin(tb);
                double L  = std::hypot(bx - ax, by - ay);
                if (L > densifyMaxChord_) {   // ★[D] 弦太长: 视为断裂, 不插值
                    deposit(ax, ay);          //     真实端点照常保留
                    continue;                 //     (bb 由下一对的 ax 或段末 deposit 覆盖)
                }
                if (densifyMode_ == "angle") {
                    // ★[E] 等角采样: 弦线两端方位 (ta, tb) 之间的每个细 bin 中心射线
                    //   与弦线求交, 交点距离就是该 bin 的样本。射线 u=(cosθ,sinθ),
                    //   弦 P(s)=A+s(B-A), 共线条件 cross(P(s),u)=0 →
                    //   s = -cross(A,u)/cross(B-A,u)。bingap<=segMaxGapBins_ 保证
                    //   跨度小, 直接用未卷绕角 ta+span 遍历。
                    deposit(ax, ay);                               // 真实端点 A
                    const int    bingap = (bb - ba + N_) % N_;
                    const double span   = bingap * angleIncrement_;
                    const int    nf     = (int)std::lround(span / dinc);
                    const double ex = bx - ax, ey = by - ay;
                    for (int f = 0; f < nf; ++f) {
                        const double th = ta + (f + 0.5) * dinc;   // 细 bin 中心
                        const double ux = std::cos(th), uy = std::sin(th);
                        const double den = ex * uy - ey * ux;       // cross(B-A, u)
                        if (std::abs(den) < 1e-12) continue;       // 弦与射线平行
                        const double sPar = -(ax * uy - ay * ux) / den;
                        if (sPar < 0.0 || sPar > 1.0) continue;    // 交点不在弦上
                        const double px = ax + sPar * ex, py = ay + sPar * ey;
                        if (px * ux + py * uy <= 0.0) continue;    // 反向射线
                        deposit(px, py);
                    }
                    continue;                                      // B 由下一对或段末补
                }
                int nsub  = std::max(1, (int)(L / densifyStep_));
                for (int t = 0; t < nsub; ++t) {
                    double u = (double)t / nsub;
                    deposit(ax + u * (bx - ax), ay + u * (by - ay));
                }
            }
            // 段末端点
            int bl = s.back();
            double tl = bl * angleIncrement_ - M_PI;
            deposit(ranges[bl] * std::cos(tl), ranges[bl] * std::sin(tl));
        }
        return dense;
    }

    // ----------------------------------------------------------------
    // max_bgsub 判决：滑窗中值背景剔除 + 残差最大值
    //   须在 bgWindow_ 已包含当前束之后调用（sonarCallback 中保证）。
    //   返回 0 表示无有效回波（caller 会 skip，该方向留 inf）。
    // ----------------------------------------------------------------
    double extractRangeBgsub(const BeamEntry& beam) const {
        const int n = (int)beam.intensities.size();
        if (n == 0 || beam.range <= 1e-6) return 0.0;
        if ((int)bgWindow_.size() < bgMinFillBeams_) return 0.0;  // 冷启动：背景不可信

        const double binSize = beam.range / (double)n;
        int kLo = (int)(rangeMin_ / binSize);
        int kHi = std::min(n - 1, (int)(rangeMax_ / binSize));
        if (kLo < 0) kLo = 0;
        if (kLo > kHi) return 0.0;

        const int w = (int)bgWindow_.size();
        std::vector<double> col(w);

        // 先算出门内每一格的残差 (以及 CFAR 模式下的稳健离散度)
        const bool cfar = residualZscore_ > 0.0;
        std::vector<double> resid(kHi - kLo + 1), zsc(cfar ? kHi - kLo + 1 : 0);
        std::vector<double> dev(w);
        for (int k = kLo; k <= kHi; ++k) {
            for (int j = 0; j < w; ++j) col[j] = bgWindow_[j][k];
            std::nth_element(col.begin(), col.begin() + w / 2, col.end());
            const double med = col[w / 2];
            resid[k - kLo] = beam.intensities[k] - med;
            if (cfar) {
                for (int j = 0; j < w; ++j) dev[j] = std::abs(bgWindow_[j][k] - med);
                std::nth_element(dev.begin(), dev.begin() + w / 2, dev.end());
                const double sigma = 1.4826 * dev[w / 2] + residualNoiseFloor_;
                zsc[k - kLo] = resid[k - kLo] / sigma;
            }
        }

        if (cfar) {   // [CFAR] 取归一化残差最大的格
            double bestZ = -1e9; int bestK = -1;
            for (int k = kLo; k <= kHi; ++k)
                if (zsc[k - kLo] > bestZ) { bestZ = zsc[k - kLo]; bestK = k; }
            if (bestK < 0 || bestZ < residualZscore_ || resid[bestK - kLo] < residualThreshold_) return 0.0;
            return beam.range * (bestK + 0.5) / (double)n;
        }

        // 门内残差最大值
        double bestResid = -1.0;
        int    bestK     = -1;
        for (int k = kLo; k <= kHi; ++k) {
            if (resid[k - kLo] > bestResid) { bestResid = resid[k - kLo]; bestK = k; }
        }
        if (bestK < 0 || bestResid < residualThreshold_) return 0.0;
        return beam.range * (bestK + 0.5) / (double)n;
    }

    // ----------------------------------------------------------------
    // 提取单个波束距离（方法分派）
    // ----------------------------------------------------------------
    double extractRange(const BeamEntry& beam) const {
        if (rangeExtractionMethod_ == "max_bgsub")
            return (beam.extractedRange > 0.0) ? beam.extractedRange : 0.0;

        if (beam.intensities.empty()) return 0.0;
        const int n = (int)beam.intensities.size();
        const std::vector<double>& I = beam.intensities;

        // 近场盲区采样数：range_min 对应多少个采样下标
        int ignore = 0;
        if (beam.range > 1e-6)
            ignore = (int)(rangeMin_ / (beam.range / (double)n));
        if (ignore < 0) ignore = 0;
        if (ignore >= n) return 0.0;

        // 默认 "max"
        int kMax = ignore;
        double vMax = I[ignore];
        for (int k = ignore + 1; k < n; ++k) {
            if (I[k] > vMax) { vMax = I[k]; kMax = k; }
        }
        if (vMax < intensityThreshold_) return 0.0;
        return beam.range * (kMax + 0.5) / n;
    }

    // ---- ROS 接口 ----
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekfSub_;
    rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr sonarSub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
    rclcpp::CallbackGroup::SharedPtr cbGrpEkf_;
    rclcpp::CallbackGroup::SharedPtr cbGrpSonar_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>       tfBroadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster_;

    // ---- EKF 缓冲 ----
    std::deque<transformationStamped> ekfBuffer_;
    std::mutex ekfMutex_;

    // ---- 声呐帧缓冲（滑窗） ----
    std::deque<BeamEntry> beamBuffer_;
    double cumAngleTotal_       = 0.0;
    double lastPublishCumAngle_ = 0.0;
    double prevBeamAngle_       = 0.0;

    // ---- max_bgsub 背景滑窗（仅 sonar 回调线程访问，免锁） ----
    std::deque<std::vector<double>> bgWindow_;
    double bgWindowRange_ = -1.0;

    // ---- 去畸变外参缓存 ----
    Eigen::Matrix4d T_BS_;
    Eigen::Matrix4d T_BS_inv_;

    // ---- 统计 ----
    long publishedCount_ = 0;
    // [Timing] 2026-09-16: per-scan front-end cost (ms), timing only
    double beamMsAccum_ = 0.0;
    long   beamsSinceScan_ = 0;
    long droppedCount_   = 0;

    // ---- 参数 ----
    std::string rangeExtractionMethod_;
    double      intensityThreshold_;
    int         bgWindowBeams_;
    int         bgMinFillBeams_;
    double      residualThreshold_;
    double      residualZscore_ = 0.0;      // [CFAR] 0 = off
    double      residualNoiseFloor_ = 3.0;

    double      angleIncrement_;
    double      rangeMin_;
    double      rangeMax_;

    double      frameAccumulateAngle_;
    double      publishIntervalAngle_;
    int         minBeamsPerScan_;
    int         minValidBinsPerScan_;

    bool        enableOutlierFilter_;
    double      outlierDistance_;
    double      outlierRangeRatio_;
    int         outlierHalfWindow_;
    int         outlierMinNeighbors_;

    bool        enableSegmentFilter_;
    int         segMaxGapBins_;
    double      segJumpPerBin_;
    int         segMinPoints_;
    bool        enableSegmentDensify_;
    double      densifyStep_;
    int         densifyOutputBins_;
    double      densifyMaxChord_;
    std::string densifyMode_ = "distance";   // [E] "distance" | "angle"

    std::string laserFrame_;
    std::string baseFrame_;
    std::string odomFrame_;
    double      sonarX_, sonarY_, sonarZ_, sonarYaw_;
    int         N_;
};

// ============================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<SonarToLaserscan>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}