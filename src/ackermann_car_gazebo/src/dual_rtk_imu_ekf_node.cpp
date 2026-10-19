#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <urdf/model.h>

#include <ackermann_car_gazebo/RtkImuEkfState.h>

namespace
{
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kGravityMps2 = 9.80665;
constexpr int kStateSize = 6;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool validQuaternion(const sensor_msgs::Imu& msg)
{
  const double norm_sq =
      msg.orientation.x * msg.orientation.x +
      msg.orientation.y * msg.orientation.y +
      msg.orientation.z * msg.orientation.z +
      msg.orientation.w * msg.orientation.w;
  return std::isfinite(norm_sq) && norm_sq > 1.0e-6;
}

Eigen::Isometry3d poseToEigen(const urdf::Pose& pose)
{
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  q.normalize();

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = q.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

double vectorDistance(const std::vector<double>& lhs, const Eigen::Vector3d& rhs)
{
  if (lhs.size() != 3) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Vector3d expected(lhs[0], lhs[1], lhs[2]);
  return (expected - rhs).norm();
}

}  // namespace

class DualRtkImuEkfNode
{
public:
  DualRtkImuEkfNode()
    : nh_(),
      pnh_("~"),
      initialized_(false),
      origin_lat_rad_(0.0),
      origin_lon_rad_(0.0),
      cos_origin_lat_(1.0)
  {
    loadParameters();
    loadGeometryFromUrdf();

    state_.setZero();
    covariance_.setZero();

    state_pub_ = nh_.advertise<ackermann_car_gazebo::RtkImuEkfState>(output_topic_, 10);
    imu_sub_ = nh_.subscribe(imu_topic_, imu_queue_size_, &DualRtkImuEkfNode::imuCallback, this);

    front_rtk_sub_.reset(new RtkSubscriber(nh_, front_rtk_topic_, sync_queue_size_));
    rear_rtk_sub_.reset(new RtkSubscriber(nh_, rear_rtk_topic_, sync_queue_size_));

    RtkSyncPolicy policy(sync_queue_size_);
    policy.setMaxIntervalDuration(ros::Duration(sync_slop_));
    rtk_sync_.reset(new RtkSynchronizer(policy));
    rtk_sync_->connectInput(*front_rtk_sub_, *rear_rtk_sub_);
    rtk_sync_->registerCallback(boost::bind(&DualRtkImuEkfNode::rtkCallback, this, _1, _2));

    ROS_INFO("dual_rtk_imu_ekf_node listening to '%s', '%s', '%s' and publishing '%s'",
             front_rtk_topic_.c_str(), rear_rtk_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str());
  }

private:
  enum class RtkQuality
  {
    Low,
    Medium,
    High
  };

  using RtkSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>;
  using RtkSynchronizer = message_filters::Synchronizer<RtkSyncPolicy>;
  using RtkSubscriber = message_filters::Subscriber<sensor_msgs::NavSatFix>;

  void loadParameters()
  {
    pnh_.param("front_rtk_topic", front_rtk_topic_, std::string("/ackermann_car/rtk_front/fix"));
    pnh_.param("rear_rtk_topic", rear_rtk_topic_, std::string("/ackermann_car/rtk_rear/fix"));
    pnh_.param("imu_topic", imu_topic_, std::string("/ackermann_car/imu/data"));
    pnh_.param("output_topic", output_topic_, std::string("/ackermann_car/ekf_state"));

    pnh_.param("map_frame", map_frame_, std::string("map"));
    pnh_.param("base_frame", base_frame_, std::string("base_link"));
    pnh_.param("imu_frame", imu_frame_, std::string("imu_link"));
    pnh_.param("front_rtk_frame", front_rtk_frame_, std::string("rtk_front_link"));
    pnh_.param("rear_rtk_frame", rear_rtk_frame_, std::string("rtk_rear_link"));
    pnh_.param("robot_description_param", robot_description_param_, std::string("/ackermann_car/robot_description"));

    double origin_latitude = 31.230416;
    double origin_longitude = 121.473701;
    pnh_.param("origin_latitude", origin_latitude, origin_latitude);
    pnh_.param("origin_longitude", origin_longitude, origin_longitude);
    pnh_.param("origin_altitude", origin_altitude_, 0.0);
    origin_lat_rad_ = origin_latitude * M_PI / 180.0;
    origin_lon_rad_ = origin_longitude * M_PI / 180.0;
    cos_origin_lat_ = std::cos(origin_lat_rad_);

    pnh_.param("sync_queue_size", sync_queue_size_, 20);
    pnh_.param("sync_slop", sync_slop_, 0.05);
    pnh_.param("imu_queue_size", imu_queue_size_, 200);
    pnh_.param("imu_buffer_duration", imu_buffer_duration_, 5.0);
    pnh_.param("imu_max_gap", imu_max_gap_, 0.10);
    pnh_.param("max_prediction_step", max_prediction_step_, 0.02);

    pnh_.param("rtk_position_std", rtk_position_std_, 0.03);
    pnh_.param("dual_rtk_yaw_std", dual_rtk_yaw_std_, 0.03);
    pnh_.param("baseline_length_tolerance", baseline_length_tolerance_, 0.10);
    pnh_.param("min_baseline_xy_length", min_baseline_xy_length_, 0.20);
    pnh_.param("use_altitude_for_baseline_gate", use_altitude_for_baseline_gate_, true);
    pnh_.param("imu_gap_process_noise_scale", imu_gap_process_noise_scale_, 25.0);
    pnh_.param("rtk_high_precision_std_threshold", rtk_high_precision_std_threshold_, 0.05);
    pnh_.param("rtk_medium_precision_std_threshold", rtk_medium_precision_std_threshold_, 0.20);
    pnh_.param("rtk_high_precision_status_min", rtk_high_precision_status_min_, 2);
    pnh_.param("rtk_direct_position_when_high_precision", rtk_direct_position_when_high_precision_, true);
    pnh_.param("use_imu_linear_acceleration", use_imu_linear_acceleration_, false);
    pnh_.param("use_gravity_compensated_acceleration", use_gravity_compensated_acceleration_, false);
    pnh_.param("rtk_velocity_filter_alpha", rtk_velocity_filter_alpha_, 0.35);
    pnh_.param("stationary_speed_threshold", stationary_speed_threshold_, 0.03);
    pnh_.param("stationary_rtk_window", stationary_rtk_window_, 1.0);

    process_noise_diag_ = readVectorParam("process_noise_diagonal", {0.02, 0.02, 0.01, 0.20, 0.20, 0.001}, kStateSize);
    initial_covariance_diag_ = readVectorParam("initial_covariance_diagonal", {0.20, 0.20, 0.10, 1.00, 1.00, 0.01}, kStateSize);
    expected_front_to_imu_ = readVectorParam("expected_front_to_imu", {0.30, 0.0, 0.14}, 3);
    expected_rear_to_imu_ = readVectorParam("expected_rear_to_imu", {-0.30, 0.0, 0.14}, 3);
    pnh_.param("expected_rtk_baseline", expected_rtk_baseline_, 0.60);
    pnh_.param("geometry_tolerance", geometry_tolerance_, 0.02);

    sync_queue_size_ = std::max(sync_queue_size_, 2);
    imu_queue_size_ = std::max(imu_queue_size_, 1);
    sync_slop_ = std::max(sync_slop_, 0.0);
    imu_buffer_duration_ = std::max(imu_buffer_duration_, 0.5);
    imu_max_gap_ = std::max(imu_max_gap_, 0.001);
    max_prediction_step_ = std::max(max_prediction_step_, 0.001);
    rtk_position_std_ = std::max(rtk_position_std_, 1.0e-6);
    dual_rtk_yaw_std_ = std::max(dual_rtk_yaw_std_, 1.0e-6);
    baseline_length_tolerance_ = std::max(baseline_length_tolerance_, 0.0);
    min_baseline_xy_length_ = std::max(min_baseline_xy_length_, 1.0e-6);
    imu_gap_process_noise_scale_ = std::max(imu_gap_process_noise_scale_, 1.0);
    rtk_high_precision_std_threshold_ = std::max(rtk_high_precision_std_threshold_, 1.0e-6);
    rtk_medium_precision_std_threshold_ =
        std::max(rtk_medium_precision_std_threshold_, rtk_high_precision_std_threshold_);
    rtk_velocity_filter_alpha_ = std::min(std::max(rtk_velocity_filter_alpha_, 0.0), 1.0);
    stationary_speed_threshold_ = std::max(stationary_speed_threshold_, 0.0);
    stationary_rtk_window_ = std::max(stationary_rtk_window_, 0.0);
  }

  std::vector<double> readVectorParam(const std::string& name,
                                      const std::vector<double>& default_value,
                                      std::size_t expected_size) const
  {
    std::vector<double> value;
    if (!pnh_.getParam(name, value)) {
      value = default_value;
    }

    if (value.size() != expected_size) {
      throw std::runtime_error("Parameter '~" + name + "' must contain " + std::to_string(expected_size) + " values.");
    }
    for (double item : value) {
      if (!std::isfinite(item)) {
        throw std::runtime_error("Parameter '~" + name + "' contains a non-finite value.");
      }
    }
    return value;
  }

  void loadGeometryFromUrdf()
  {
    std::string robot_description;
    if (!nh_.getParam(robot_description_param_, robot_description)) {
      ROS_WARN("Could not read '%s'; falling back to '/robot_description'", robot_description_param_.c_str());
      if (!nh_.getParam("/robot_description", robot_description)) {
        throw std::runtime_error("Could not read robot_description from '" + robot_description_param_ + "' or '/robot_description'.");
      }
    }

    urdf::Model model;
    if (!model.initString(robot_description)) {
      throw std::runtime_error("Failed to parse robot_description as URDF.");
    }

    imu_in_base_ = transformBetweenLinks(model, base_frame_, imu_frame_).translation();
    rtk_front_in_base_ = transformBetweenLinks(model, base_frame_, front_rtk_frame_).translation();
    rtk_rear_in_base_ = transformBetweenLinks(model, base_frame_, rear_rtk_frame_).translation();
    rtk_baseline_in_base_ = rtk_front_in_base_ - rtk_rear_in_base_;

    const double front_error = vectorDistance(expected_front_to_imu_, rtk_front_in_base_ - imu_in_base_);
    const double rear_error = vectorDistance(expected_rear_to_imu_, rtk_rear_in_base_ - imu_in_base_);
    const double baseline_error = std::abs(rtk_baseline_in_base_.norm() - expected_rtk_baseline_);

    if (front_error > geometry_tolerance_ || rear_error > geometry_tolerance_ || baseline_error > geometry_tolerance_) {
      std::ostringstream stream;
      stream << "URDF geometry does not match YAML expectations. "
             << "front_to_imu_error=" << front_error
             << ", rear_to_imu_error=" << rear_error
             << ", baseline_error=" << baseline_error
             << ", tolerance=" << geometry_tolerance_;
      throw std::runtime_error(stream.str());
    }

    if (rtk_baseline_in_base_.head<2>().norm() < min_baseline_xy_length_) {
      throw std::runtime_error("RTK baseline xy length from URDF is too small for yaw estimation.");
    }

    ROS_INFO("URDF geometry: imu=(%.3f %.3f %.3f), rtk_front=(%.3f %.3f %.3f), rtk_rear=(%.3f %.3f %.3f), baseline=%.3f m",
             imu_in_base_.x(), imu_in_base_.y(), imu_in_base_.z(),
             rtk_front_in_base_.x(), rtk_front_in_base_.y(), rtk_front_in_base_.z(),
             rtk_rear_in_base_.x(), rtk_rear_in_base_.y(), rtk_rear_in_base_.z(),
             rtk_baseline_in_base_.norm());
  }

  Eigen::Isometry3d transformBetweenLinks(const urdf::Model& model,
                                          const std::string& parent_frame,
                                          const std::string& child_frame) const
  {
    if (parent_frame == child_frame) {
      return Eigen::Isometry3d::Identity();
    }

    std::vector<Eigen::Isometry3d> parent_to_child_chain;
    std::string current_frame = child_frame;
    while (current_frame != parent_frame) {
      urdf::LinkConstSharedPtr link = model.getLink(current_frame);
      if (!link) {
        throw std::runtime_error("URDF link '" + current_frame + "' was not found.");
      }
      if (!link->parent_joint) {
        throw std::runtime_error("URDF link '" + current_frame + "' is not connected to parent '" + parent_frame + "'.");
      }
      const urdf::JointConstSharedPtr joint = link->parent_joint;
      if (joint->type != urdf::Joint::FIXED) {
        throw std::runtime_error("URDF joint '" + joint->name + "' on the sensor chain is not fixed.");
      }

      parent_to_child_chain.push_back(poseToEigen(joint->parent_to_joint_origin_transform));
      current_frame = joint->parent_link_name;
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    for (auto iter = parent_to_child_chain.rbegin(); iter != parent_to_child_chain.rend(); ++iter) {
      transform = transform * (*iter);
    }
    return transform;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    try {
      imuCallbackImpl(msg);
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "IMU callback failed: %s", error.what());
    }
  }

  void imuCallbackImpl(const sensor_msgs::ImuConstPtr& msg)
  {
    if (msg->header.stamp.isZero()) {
      ROS_WARN_THROTTLE(5.0, "Ignoring IMU message with zero timestamp.");
      return;
    }

    if (!imuBufferAccepts(*msg)) {
      ROS_WARN_THROTTLE(5.0, "Ignoring IMU message with non-finite required fields.");
      return;
    }

    if (imu_buffer_.empty() || msg->header.stamp >= imu_buffer_.back()->header.stamp) {
      imu_buffer_.push_back(msg);
    } else {
      auto insert_at = std::upper_bound(
          imu_buffer_.begin(), imu_buffer_.end(), msg->header.stamp,
          [](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& item) {
            return stamp < item->header.stamp;
          });
      imu_buffer_.insert(insert_at, msg);
    }

    trimImuBuffer(historyCutoff(msg->header.stamp));
  }

  bool imuBufferAccepts(const sensor_msgs::Imu& msg) const
  {
    if (!isFinite(msg.angular_velocity.z)) {
      return false;
    }

    if (!use_imu_linear_acceleration_) {
      return true;
    }

    if (!isFinite(msg.linear_acceleration.x) || !isFinite(msg.linear_acceleration.y)) {
      return false;
    }

    return !use_gravity_compensated_acceleration_ || isFinite(msg.linear_acceleration.z);
  }

  void rtkCallback(const sensor_msgs::NavSatFixConstPtr& front, const sensor_msgs::NavSatFixConstPtr& rear)
  {
    try {
      rtkCallbackImpl(front, rear);
    } catch (const std::exception& error) {
      ROS_ERROR_THROTTLE(1.0, "RTK callback failed: %s", error.what());
    }
  }

  void rtkCallbackImpl(const sensor_msgs::NavSatFixConstPtr& front, const sensor_msgs::NavSatFixConstPtr& rear)
  {
    if (!validateFix(*front, "front") || !validateFix(*rear, "rear")) {
      return;
    }

    const double stamp_delta = std::abs((front->header.stamp - rear->header.stamp).toSec());
    if (stamp_delta > sync_slop_) {
      ROS_WARN_THROTTLE(2.0, "Dropping RTK pair with timestamp delta %.3f s > sync_slop %.3f s", stamp_delta, sync_slop_);
      return;
    }

    const ros::Time fusion_stamp = (front->header.stamp >= rear->header.stamp) ? front->header.stamp : rear->header.stamp;
    Eigen::Vector3d front_enu = navSatToEnu(*front);
    Eigen::Vector3d rear_enu = navSatToEnu(*rear);

    Eigen::Vector2d base_position;
    double measured_yaw = 0.0;
    if (!computeRtkMeasurement(front_enu, rear_enu, base_position, measured_yaw)) {
      return;
    }
    const RtkQuality rtk_quality = classifyRtkQuality(*front, *rear);

    if (!initialized_) {
      if (rtk_quality == RtkQuality::Low) {
        ROS_WARN_THROTTLE(2.0, "Waiting for medium/high precision RTK before initializing EKF.");
        return;
      }
      initializeFilter(fusion_stamp, base_position, measured_yaw);
      updateRtkVelocity(fusion_stamp, base_position);
      publishState(fusion_stamp);
      return;
    }

    if (fusion_stamp < last_update_stamp_) {
      ROS_WARN_THROTTLE(2.0, "Dropping out-of-order RTK update: %.6f < %.6f",
                        fusion_stamp.toSec(), last_update_stamp_.toSec());
      return;
    }

    if (rtk_quality == RtkQuality::Low) {
      const double prediction_dt = (fusion_stamp - last_update_stamp_).toSec();
      predictTo(fusion_stamp);
      inflateCovarianceForDroppedRtk(prediction_dt);
      last_update_stamp_ = fusion_stamp;
      publishState(fusion_stamp);
      ROS_WARN_THROTTLE(2.0, "Dropping RTK correction because covariance/status indicates low precision; publishing IMU-only prediction.");
      return;
    }

    predictTo(fusion_stamp);
    updateRtkVelocity(fusion_stamp, base_position);
    if (rtk_quality == RtkQuality::High && rtk_direct_position_when_high_precision_) {
      applyHighPrecisionRtk(base_position, measured_yaw);
    } else {
      updateWithRtk(base_position, measured_yaw);
    }
    last_update_stamp_ = fusion_stamp;
    publishState(fusion_stamp);
  }

  bool validateFix(const sensor_msgs::NavSatFix& fix, const std::string& label) const
  {
    if (fix.header.stamp.isZero()) {
      ROS_WARN_THROTTLE(5.0, "Dropping %s RTK fix with zero timestamp.", label.c_str());
      return false;
    }
    if (fix.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      ROS_WARN_THROTTLE(5.0, "Dropping %s RTK fix because status is %d.", label.c_str(), fix.status.status);
      return false;
    }
    if (!isFinite(fix.latitude) || !isFinite(fix.longitude) || !isFinite(fix.altitude)) {
      ROS_WARN_THROTTLE(5.0, "Dropping %s RTK fix with non-finite latitude, longitude, or altitude.", label.c_str());
      return false;
    }
    return true;
  }

  RtkQuality classifyRtkQuality(const sensor_msgs::NavSatFix& front,
                                const sensor_msgs::NavSatFix& rear) const
  {
    if (front.status.status < sensor_msgs::NavSatStatus::STATUS_FIX ||
        rear.status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      return RtkQuality::Low;
    }

    const double front_std = horizontalStd(front);
    const double rear_std = horizontalStd(rear);
    if (!std::isfinite(front_std) || !std::isfinite(rear_std)) {
      return RtkQuality::Low;
    }

    const double worst_std = std::max(front_std, rear_std);
    if (front.status.status >= rtk_high_precision_status_min_ &&
        rear.status.status >= rtk_high_precision_status_min_ &&
        worst_std <= rtk_high_precision_std_threshold_) {
      return RtkQuality::High;
    }
    if (worst_std <= rtk_medium_precision_std_threshold_) {
      return RtkQuality::Medium;
    }
    return RtkQuality::Low;
  }

  double horizontalStd(const sensor_msgs::NavSatFix& fix) const
  {
    if (fix.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double var_x = fix.position_covariance[0];
    const double var_y = fix.position_covariance[4];
    if (!std::isfinite(var_x) || !std::isfinite(var_y) || var_x < 0.0 || var_y < 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(std::max(var_x, var_y));
  }

  Eigen::Vector3d navSatToEnu(const sensor_msgs::NavSatFix& fix) const
  {
    const double lat_rad = fix.latitude * M_PI / 180.0;
    const double lon_rad = fix.longitude * M_PI / 180.0;
    const double x = (lon_rad - origin_lon_rad_) * kEarthRadiusM * cos_origin_lat_;
    const double y = (lat_rad - origin_lat_rad_) * kEarthRadiusM;
    const double z = fix.altitude - origin_altitude_;
    return Eigen::Vector3d(x, y, z);
  }

  bool computeRtkMeasurement(const Eigen::Vector3d& front_enu,
                             const Eigen::Vector3d& rear_enu,
                             Eigen::Vector2d& base_position,
                             double& measured_yaw) const
  {
    const Eigen::Vector3d baseline_enu = front_enu - rear_enu;
    const double baseline_xy_length = baseline_enu.head<2>().norm();
    if (baseline_xy_length < min_baseline_xy_length_) {
      ROS_WARN_THROTTLE(2.0, "Dropping RTK pair with xy baseline %.3f m < %.3f m",
                        baseline_xy_length, min_baseline_xy_length_);
      return false;
    }

    const double measured_baseline_length = use_altitude_for_baseline_gate_ ? baseline_enu.norm() : baseline_xy_length;
    const double expected_baseline_length = use_altitude_for_baseline_gate_ ?
        rtk_baseline_in_base_.norm() : rtk_baseline_in_base_.head<2>().norm();
    const double baseline_error = std::abs(measured_baseline_length - expected_baseline_length);
    if (baseline_error > baseline_length_tolerance_) {
      ROS_WARN_THROTTLE(2.0, "Dropping RTK pair with baseline error %.3f m > %.3f m",
                        baseline_error, baseline_length_tolerance_);
      return false;
    }

    const double map_baseline_yaw = std::atan2(baseline_enu.y(), baseline_enu.x());
    const double body_baseline_yaw = std::atan2(rtk_baseline_in_base_.y(), rtk_baseline_in_base_.x());
    measured_yaw = normalizeAngle(map_baseline_yaw - body_baseline_yaw);

    const double c = std::cos(measured_yaw);
    const double s = std::sin(measured_yaw);
    Eigen::Matrix2d rotation;
    rotation << c, -s, s, c;

    const Eigen::Vector2d front_base =
        front_enu.head<2>() - rotation * rtk_front_in_base_.head<2>();
    const Eigen::Vector2d rear_base =
        rear_enu.head<2>() - rotation * rtk_rear_in_base_.head<2>();
    base_position = 0.5 * (front_base + rear_base);
    return true;
  }

  void initializeFilter(const ros::Time& stamp, const Eigen::Vector2d& base_position, double measured_yaw)
  {
    state_.setZero();
    state_(0) = base_position.x();
    state_(1) = base_position.y();
    state_(2) = normalizeAngle(measured_yaw);

    covariance_.setZero();
    for (int i = 0; i < kStateSize; ++i) {
      covariance_(i, i) = initial_covariance_diag_[i];
    }

    initialized_ = true;
    last_update_stamp_ = stamp;
    ROS_INFO("Initialized dual RTK + IMU EKF at x=%.3f, y=%.3f, yaw=%.3f rad",
             state_(0), state_(1), state_(2));
  }

  void applyHighPrecisionRtk(const Eigen::Vector2d& base_position, double measured_yaw)
  {
    const double yaw_residual = normalizeAngle(measured_yaw - state_(2));
    const double yaw_innovation_var =
        covariance_(2, 2) + dual_rtk_yaw_std_ * dual_rtk_yaw_std_;
    if (std::isfinite(yaw_innovation_var) && yaw_innovation_var > 1.0e-9) {
      const double bias_step = covariance_(5, 2) / yaw_innovation_var * yaw_residual;
      if (std::isfinite(bias_step)) {
        state_(5) += std::min(std::max(bias_step, -0.05), 0.05);
      }
    }

    state_(0) = base_position.x();
    state_(1) = base_position.y();
    state_(2) = normalizeAngle(measured_yaw);

    const double position_var =
        rtk_high_precision_std_threshold_ * rtk_high_precision_std_threshold_;
    const double yaw_var = dual_rtk_yaw_std_ * dual_rtk_yaw_std_;
    for (int index = 0; index < 3; ++index) {
      covariance_.row(index).setZero();
      covariance_.col(index).setZero();
    }
    covariance_(0, 0) = position_var;
    covariance_(1, 1) = position_var;
    covariance_(2, 2) = yaw_var;

    const double velocity_std = std::max(stationary_speed_threshold_, 0.01);
    const double velocity_var = velocity_std * velocity_std;
    covariance_(3, 3) = std::min(covariance_(3, 3), velocity_var);
    covariance_(4, 4) = std::min(covariance_(4, 4), velocity_var);
  }

  void updateRtkVelocity(const ros::Time& stamp, const Eigen::Vector2d& base_position)
  {
    rtk_position_history_.push_back(std::make_pair(stamp, base_position));
    trimRtkPositionHistory(stamp);

    if (last_rtk_velocity_stamp_.isZero()) {
      state_(3) = 0.0;
      state_(4) = 0.0;
      last_rtk_velocity_stamp_ = stamp;
      last_rtk_velocity_position_ = base_position;
      return;
    }

    const double dt = (stamp - last_rtk_velocity_stamp_).toSec();
    if (dt > 1.0e-4) {
      const Eigen::Vector2d raw_velocity = (base_position - last_rtk_velocity_position_) / dt;
      const double alpha = rtk_velocity_filter_alpha_;
      state_(3) = alpha * raw_velocity.x() + (1.0 - alpha) * state_(3);
      state_(4) = alpha * raw_velocity.y() + (1.0 - alpha) * state_(4);
    }

    last_rtk_velocity_stamp_ = stamp;
    last_rtk_velocity_position_ = base_position;

    if (isRtkStationary()) {
      state_(3) = 0.0;
      state_(4) = 0.0;
    }
  }

  void trimRtkPositionHistory(const ros::Time& stamp)
  {
    const ros::Time oldest = rtkHistoryCutoff(stamp);
    while (!rtk_position_history_.empty() && rtk_position_history_.front().first < oldest) {
      rtk_position_history_.pop_front();
    }
  }

  bool isRtkStationary() const
  {
    if (rtk_position_history_.size() < 2 || stationary_rtk_window_ <= 0.0) {
      return false;
    }
    const double window_dt =
        (rtk_position_history_.back().first - rtk_position_history_.front().first).toSec();
    if (window_dt < stationary_rtk_window_ * 0.8) {
      return false;
    }

    Eigen::Vector2d min_position = rtk_position_history_.front().second;
    Eigen::Vector2d max_position = rtk_position_history_.front().second;
    for (const auto& item : rtk_position_history_) {
      min_position = min_position.cwiseMin(item.second);
      max_position = max_position.cwiseMax(item.second);
    }

    const double displacement_bound = std::max(stationary_speed_threshold_ * stationary_rtk_window_,
                                               rtk_high_precision_std_threshold_);
    return (max_position - min_position).norm() <= displacement_bound;
  }

  void predictTo(const ros::Time& target_stamp)
  {
    if (target_stamp <= last_update_stamp_) {
      return;
    }

    trimImuBuffer(historyCutoff(last_update_stamp_));

    ros::Time current_stamp = last_update_stamp_;
    sensor_msgs::ImuConstPtr active_imu = latestImuAtOrBefore(current_stamp);
    bool saw_gap = false;

    for (const auto& imu : imu_buffer_) {
      if (imu->header.stamp <= current_stamp) {
        continue;
      }
      if (imu->header.stamp > target_stamp) {
        break;
      }

      saw_gap = predictSegment(current_stamp, imu->header.stamp, active_imu) || saw_gap;
      current_stamp = imu->header.stamp;
      active_imu = imu;
    }

    saw_gap = predictSegment(current_stamp, target_stamp, active_imu) || saw_gap;
    state_(2) = normalizeAngle(state_(2));

    if (saw_gap) {
      ROS_WARN_THROTTLE(2.0, "IMU gap or stale IMU sample detected during EKF prediction; inflated process noise was used.");
    }
  }

  sensor_msgs::ImuConstPtr latestImuAtOrBefore(const ros::Time& stamp) const
  {
    sensor_msgs::ImuConstPtr latest;
    for (const auto& imu : imu_buffer_) {
      if (imu->header.stamp <= stamp) {
        latest = imu;
      } else {
        break;
      }
    }
    return latest;
  }

  bool predictSegment(const ros::Time& start_stamp,
                      const ros::Time& end_stamp,
                      const sensor_msgs::ImuConstPtr& imu)
  {
    double remaining_dt = (end_stamp - start_stamp).toSec();
    if (remaining_dt <= 0.0) {
      return false;
    }

    const bool imu_stale = !imu || ((start_stamp - imu->header.stamp).toSec() > imu_max_gap_);
    bool inflated_noise = imu_stale;

    while (remaining_dt > 1.0e-9) {
      const double dt = std::min(remaining_dt, max_prediction_step_);
      predictSmallStep(dt, imu_stale ? sensor_msgs::ImuConstPtr() : imu, inflated_noise);
      remaining_dt -= dt;
    }
    return inflated_noise;
  }

  void inflateCovarianceForDroppedRtk(double dt)
  {
    const double noise_scale = imu_gap_process_noise_scale_ * std::max(dt, max_prediction_step_);
    for (int i = 0; i < kStateSize; ++i) {
      covariance_(i, i) += std::max(process_noise_diag_[i], 0.0) * noise_scale;
    }
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  void predictSmallStep(double dt, const sensor_msgs::ImuConstPtr& imu, bool inflated_noise)
  {
    double gyro_z = state_(5);
    double ax_map = 0.0;
    double ay_map = 0.0;
    double dax_dyaw = 0.0;
    double day_dyaw = 0.0;

    if (imu) {
      gyro_z = imu->angular_velocity.z;

      if (use_imu_linear_acceleration_) {
        if (use_gravity_compensated_acceleration_ && validQuaternion(*imu) &&
            isFinite(imu->linear_acceleration.x) &&
            isFinite(imu->linear_acceleration.y) &&
            isFinite(imu->linear_acceleration.z)) {
          Eigen::Quaterniond q(imu->orientation.w,
                               imu->orientation.x,
                               imu->orientation.y,
                               imu->orientation.z);
          q.normalize();
          const Eigen::Vector3d measured_body_accel(imu->linear_acceleration.x,
                                                    imu->linear_acceleration.y,
                                                    imu->linear_acceleration.z);
          const Eigen::Vector3d gravity_compensated_map_accel =
              q * measured_body_accel - Eigen::Vector3d(0.0, 0.0, kGravityMps2);
          ax_map = gravity_compensated_map_accel.x();
          ay_map = gravity_compensated_map_accel.y();
        } else if (!use_gravity_compensated_acceleration_ &&
                   isFinite(imu->linear_acceleration.x) &&
                   isFinite(imu->linear_acceleration.y)) {
          const double yaw = state_(2);
          const double c = std::cos(yaw);
          const double s = std::sin(yaw);
          const double ax_body = imu->linear_acceleration.x;
          const double ay_body = imu->linear_acceleration.y;
          ax_map = c * ax_body - s * ay_body;
          ay_map = s * ax_body + c * ay_body;
          dax_dyaw = -s * ax_body - c * ay_body;
          day_dyaw = c * ax_body - s * ay_body;
        } else {
          ROS_WARN_THROTTLE(5.0, "IMU acceleration integration requested, but orientation/acceleration is invalid; using gyro-only prediction.");
        }
      }
    }

    state_(0) += state_(3) * dt + 0.5 * ax_map * dt * dt;
    state_(1) += state_(4) * dt + 0.5 * ay_map * dt * dt;
    state_(2) = normalizeAngle(state_(2) + (gyro_z - state_(5)) * dt);
    state_(3) += ax_map * dt;
    state_(4) += ay_map * dt;

    Eigen::Matrix<double, kStateSize, kStateSize> F =
        Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
    F(0, 2) = 0.5 * dax_dyaw * dt * dt;
    F(0, 3) = dt;
    F(1, 2) = 0.5 * day_dyaw * dt * dt;
    F(1, 4) = dt;
    F(2, 5) = -dt;
    F(3, 2) = dax_dyaw * dt;
    F(4, 2) = day_dyaw * dt;

    Eigen::Matrix<double, kStateSize, kStateSize> Q =
        Eigen::Matrix<double, kStateSize, kStateSize>::Zero();
    const double noise_scale = (inflated_noise ? imu_gap_process_noise_scale_ : 1.0) * dt;
    for (int i = 0; i < kStateSize; ++i) {
      Q(i, i) = std::max(process_noise_diag_[i], 0.0) * noise_scale;
    }

    covariance_ = F * covariance_ * F.transpose() + Q;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  void updateWithRtk(const Eigen::Vector2d& base_position, double measured_yaw)
  {
    Eigen::Matrix<double, 3, 1> residual;
    residual << base_position.x() - state_(0),
                base_position.y() - state_(1),
                normalizeAngle(measured_yaw - state_(2));

    Eigen::Matrix<double, 3, kStateSize> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = rtk_position_std_ * rtk_position_std_;
    R(1, 1) = rtk_position_std_ * rtk_position_std_;
    R(2, 2) = dual_rtk_yaw_std_ * dual_rtk_yaw_std_;

    const Eigen::Matrix3d S = H * covariance_ * H.transpose() + R;
    Eigen::LDLT<Eigen::Matrix3d> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      ROS_WARN_THROTTLE(1.0, "Skipping RTK update because innovation covariance decomposition failed.");
      return;
    }
    const Eigen::Matrix<double, kStateSize, 3> K =
        (ldlt.solve(H * covariance_)).transpose();
    state_ += K * residual;
    state_(2) = normalizeAngle(state_(2));

    const Eigen::Matrix<double, kStateSize, kStateSize> I =
        Eigen::Matrix<double, kStateSize, kStateSize>::Identity();
    covariance_ = (I - K * H) * covariance_ * (I - K * H).transpose() + K * R * K.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  void publishState(const ros::Time& stamp) const
  {
    for (int i = 0; i < kStateSize; ++i) {
      if (!std::isfinite(state_(i))) {
        ROS_ERROR_THROTTLE(1.0, "EKF state contains a non-finite value at index %d; skipping publish.", i);
        return;
      }
    }

    ackermann_car_gazebo::RtkImuEkfState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.x = state_(0);
    msg.y = state_(1);
    msg.yaw = state_(2);
    msg.vx = state_(3);
    msg.vy = state_(4);
    state_pub_.publish(msg);
  }

  void trimImuBuffer(const ros::Time& oldest_stamp)
  {
    while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp < oldest_stamp) {
      imu_buffer_.pop_front();
    }
  }

  ros::Time historyCutoff(const ros::Time& stamp) const
  {
    const double cutoff = stamp.toSec() - imu_buffer_duration_;
    if (cutoff <= 0.0) {
      return ros::Time(0);
    }
    return ros::Time(cutoff);
  }

  ros::Time rtkHistoryCutoff(const ros::Time& stamp) const
  {
    const double cutoff = stamp.toSec() - stationary_rtk_window_;
    if (cutoff <= 0.0) {
      return ros::Time(0);
    }
    return ros::Time(cutoff);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher state_pub_;
  ros::Subscriber imu_sub_;

  std::unique_ptr<RtkSubscriber> front_rtk_sub_;
  std::unique_ptr<RtkSubscriber> rear_rtk_sub_;
  std::unique_ptr<RtkSynchronizer> rtk_sync_;

  std::deque<sensor_msgs::ImuConstPtr> imu_buffer_;
  std::deque<std::pair<ros::Time, Eigen::Vector2d>> rtk_position_history_;

  std::string front_rtk_topic_;
  std::string rear_rtk_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string front_rtk_frame_;
  std::string rear_rtk_frame_;
  std::string robot_description_param_;

  int sync_queue_size_;
  int imu_queue_size_;
  double sync_slop_;
  double imu_buffer_duration_;
  double imu_max_gap_;
  double max_prediction_step_;
  double rtk_position_std_;
  double dual_rtk_yaw_std_;
  double baseline_length_tolerance_;
  double min_baseline_xy_length_;
  double imu_gap_process_noise_scale_;
  double rtk_high_precision_std_threshold_;
  double rtk_medium_precision_std_threshold_;
  int rtk_high_precision_status_min_;
  bool rtk_direct_position_when_high_precision_;
  bool use_imu_linear_acceleration_;
  bool use_gravity_compensated_acceleration_;
  double rtk_velocity_filter_alpha_;
  double stationary_speed_threshold_;
  double stationary_rtk_window_;
  bool use_altitude_for_baseline_gate_;

  double origin_lat_rad_;
  double origin_lon_rad_;
  double origin_altitude_;
  double cos_origin_lat_;

  std::vector<double> process_noise_diag_;
  std::vector<double> initial_covariance_diag_;
  std::vector<double> expected_front_to_imu_;
  std::vector<double> expected_rear_to_imu_;
  double expected_rtk_baseline_;
  double geometry_tolerance_;

  Eigen::Vector3d imu_in_base_;
  Eigen::Vector3d rtk_front_in_base_;
  Eigen::Vector3d rtk_rear_in_base_;
  Eigen::Vector3d rtk_baseline_in_base_;

  Eigen::Matrix<double, kStateSize, 1> state_;
  Eigen::Matrix<double, kStateSize, kStateSize> covariance_;
  bool initialized_;
  ros::Time last_update_stamp_;
  ros::Time last_rtk_velocity_stamp_;
  Eigen::Vector2d last_rtk_velocity_position_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_rtk_imu_ekf");
  try {
    DualRtkImuEkfNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("dual_rtk_imu_ekf_node failed: %s", error.what());
    return 1;
  }
  return 0;
}
