/**
 * *********************************************************
 *
 * @file: mpc_controller_node.cpp
 * @brief: Standalone ROS MPC path tracker using OSQP.
 *
 * MPC/QP formulation adapted from Yang Haodong's MPC local planner:
 *   mpc_planner.cpp, 2024-01-31, version 1.0.
 *
 * This node keeps the ackermann_car_gazebo GUI/topic interface and removes the
 * original move_base/nav_core plugin dependencies.
 *
 * ********************************************************
 */

#include <ackermann_car_gazebo/RtkImuEkfState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{

constexpr int kStateDim = 3;
constexpr int kControlDim = 2;

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

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

double quaternionYaw(const geometry_msgs::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool isFiniteValue(const double value)
{
  return std::isfinite(value);
}

double distance2d(const double ax, const double ay, const double bx, const double by)
{
  return std::hypot(ax - bx, ay - by);
}

struct State2d
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
  double yaw_rate = 0.0;
  ros::WallTime receipt_time;
  bool valid = false;
};

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double s = 0.0;
};

struct Projection
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double s = 0.0;
  double distance = std::numeric_limits<double>::infinity();
  std::size_t segment_index = 0;
};

struct MpcConfig
{
  double control_dt = 0.10;
  int predict_time_domain = 10;
  int control_time_domain = 4;
  double q_matrix_x = 12.0;
  double q_matrix_y = 12.0;
  double q_matrix_theta = 3.0;
  double r_matrix_v = 0.8;
  double r_matrix_w = 0.8;
  double min_linear_velocity = 0.0;
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 1.2;
  double delta_linear_velocity_min = -0.20;
  double delta_linear_velocity_max = 0.12;
  double delta_angular_velocity_min = -0.30;
  double delta_angular_velocity_max = 0.30;
};

struct MpcResult
{
  bool solved = false;
  std::string status;
  Eigen::Vector2d control = Eigen::Vector2d::Zero();
};

class MpcController
{
public:
  void configure(const MpcConfig& config)
  {
    config_ = config;
    q_matrix_ = Eigen::Matrix3d::Zero();
    q_matrix_(0, 0) = config_.q_matrix_x;
    q_matrix_(1, 1) = config_.q_matrix_y;
    q_matrix_(2, 2) = config_.q_matrix_theta;

    r_matrix_ = Eigen::Matrix2d::Zero();
    r_matrix_(0, 0) = config_.r_matrix_v;
    r_matrix_(1, 1) = config_.r_matrix_w;
    previous_control_ = Eigen::Vector2d::Zero();
  }

  void reset()
  {
    previous_control_ = Eigen::Vector2d::Zero();
  }

  MpcResult compute(const Eigen::Vector3d& state,
                    const Eigen::Vector3d& desired_state,
                    const Eigen::Vector2d& reference_control,
                    const double max_linear_velocity)
  {
    MpcResult result;
    result.control = reference_control;
    const double feasible_deceleration_upper =
        previous_control_[0] + config_.delta_linear_velocity_min;
    const double linear_upper =
        clamp(std::max(max_linear_velocity, feasible_deceleration_upper),
              config_.min_linear_velocity,
              config_.max_linear_velocity);

    Eigen::VectorXd x = Eigen::VectorXd(kStateDim + kControlDim);
    x.topLeftCorner(kStateDim, 1) = state - desired_state;
    x[2] = normalizeAngle(x[2]);
    x.bottomLeftCorner(kControlDim, 1) = previous_control_ - reference_control;

    Eigen::Matrix3d a_original = Eigen::Matrix3d::Identity();
    a_original(0, 2) = -reference_control[0] * std::sin(desired_state[2]) * config_.control_dt;
    a_original(1, 2) = reference_control[0] * std::cos(desired_state[2]) * config_.control_dt;

    Eigen::MatrixXd b_original = Eigen::MatrixXd::Zero(kStateDim, kControlDim);
    b_original(0, 0) = std::cos(desired_state[2]) * config_.control_dt;
    b_original(1, 0) = std::sin(desired_state[2]) * config_.control_dt;
    b_original(2, 1) = config_.control_dt;

    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(kStateDim + kControlDim, kStateDim + kControlDim);
    a.topLeftCorner(kStateDim, kStateDim) = a_original;
    a.topRightCorner(kStateDim, kControlDim) = b_original;
    a.bottomRightCorner(kControlDim, kControlDim) = Eigen::Matrix2d::Identity();

    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(kStateDim + kControlDim, kControlDim);
    b.topLeftCorner(kStateDim, kControlDim) = b_original;
    b.bottomLeftCorner(kControlDim, kControlDim) = Eigen::Matrix2d::Identity();

    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(kStateDim, kStateDim + kControlDim);
    c.topLeftCorner(kStateDim, kStateDim) = Eigen::Matrix3d::Identity();

    Eigen::MatrixPower<Eigen::MatrixXd> a_power(a);
    Eigen::MatrixXd s_x =
        Eigen::MatrixXd::Zero(kStateDim * config_.predict_time_domain, kStateDim + kControlDim);
    for (int i = 0; i < config_.predict_time_domain; ++i) {
      s_x.middleRows(kStateDim * i, kStateDim) = c * a_power(i + 1);
    }

    Eigen::MatrixXd s_u =
        Eigen::MatrixXd::Zero(kStateDim * config_.predict_time_domain,
                              kControlDim * config_.control_time_domain);
    for (int i = 0; i < config_.predict_time_domain; ++i) {
      for (int j = 0; j < config_.control_time_domain; ++j) {
        if (j <= i) {
          s_u.block(kStateDim * i, kControlDim * j, kStateDim, kControlDim) =
              c * a_power(i - j) * b;
        }
      }
    }

    const Eigen::VectorXd y_reference =
        Eigen::VectorXd::Zero(kStateDim * config_.predict_time_domain);
    const Eigen::MatrixXd q_weight =
        Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(config_.predict_time_domain,
                                                          config_.predict_time_domain),
                                q_matrix_);
    const Eigen::MatrixXd r_weight =
        Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(config_.control_time_domain,
                                                          config_.control_time_domain),
                                r_matrix_);
    const Eigen::MatrixXd p = s_u.transpose() * q_weight * s_u + r_weight;
    const Eigen::VectorXd q = s_u.transpose() * q_weight * (s_x * x - y_reference);

    const Eigen::Vector2d u_min(config_.min_linear_velocity, -config_.max_angular_velocity);
    const Eigen::Vector2d u_max(linear_upper, config_.max_angular_velocity);
    const Eigen::Vector2d du_min(config_.delta_linear_velocity_min,
                                 config_.delta_angular_velocity_min);
    const Eigen::Vector2d du_max(config_.delta_linear_velocity_max,
                                 config_.delta_angular_velocity_max);

    const Eigen::VectorXd u_min_all =
        Eigen::kroneckerProduct(Eigen::VectorXd::Ones(config_.control_time_domain), u_min);
    const Eigen::VectorXd u_max_all =
        Eigen::kroneckerProduct(Eigen::VectorXd::Ones(config_.control_time_domain), u_max);
    const Eigen::VectorXd previous_control_all =
        Eigen::kroneckerProduct(Eigen::VectorXd::Ones(config_.control_time_domain), previous_control_);
    const Eigen::VectorXd du_min_all =
        Eigen::kroneckerProduct(Eigen::VectorXd::Ones(config_.control_time_domain), du_min);
    const Eigen::VectorXd du_max_all =
        Eigen::kroneckerProduct(Eigen::VectorXd::Ones(config_.control_time_domain), du_max);

    Eigen::VectorXd lower =
        Eigen::VectorXd::Zero(2 * kControlDim * config_.control_time_domain);
    Eigen::VectorXd upper =
        Eigen::VectorXd::Zero(2 * kControlDim * config_.control_time_domain);
    lower.topRows(kControlDim * config_.control_time_domain) =
        u_min_all - previous_control_all;
    lower.bottomRows(kControlDim * config_.control_time_domain) = du_min_all;
    upper.topRows(kControlDim * config_.control_time_domain) =
        u_max_all - previous_control_all;
    upper.bottomRows(kControlDim * config_.control_time_domain) = du_max_all;

    std::vector<c_float> p_data;
    std::vector<c_int> p_indices;
    std::vector<c_int> p_indptr;
    int p_nnz = 0;
    for (int col = 0; col < kControlDim * config_.control_time_domain; ++col) {
      p_indptr.push_back(p_nnz);
      for (int row = 0; row <= col; ++row) {
        p_data.push_back(static_cast<c_float>(p(row, col)));
        p_indices.push_back(static_cast<c_int>(row));
        ++p_nnz;
      }
    }
    p_indptr.push_back(p_nnz);

    std::vector<c_float> a_data;
    std::vector<c_int> a_indices;
    std::vector<c_int> a_indptr;
    int a_nnz = 0;
    a_indptr.push_back(a_nnz);
    for (int j = 0; j < config_.control_time_domain; ++j) {
      for (int n = 0; n < kControlDim; ++n) {
        for (int row = kControlDim * j + n; row < kControlDim * config_.control_time_domain;
             row += kControlDim) {
          a_data.push_back(1.0);
          a_indices.push_back(static_cast<c_int>(row));
          ++a_nnz;
        }
        a_data.push_back(1.0);
        a_indices.push_back(static_cast<c_int>(kControlDim * config_.control_time_domain +
                                               kControlDim * j + n));
        ++a_nnz;
        a_indptr.push_back(a_nnz);
      }
    }

    std::vector<c_float> q_data;
    q_data.reserve(kControlDim * config_.control_time_domain);
    for (int row = 0; row < kControlDim * config_.control_time_domain; ++row) {
      q_data.push_back(static_cast<c_float>(q(row)));
    }

    std::vector<c_float> lower_bounds;
    std::vector<c_float> upper_bounds;
    lower_bounds.reserve(2 * kControlDim * config_.control_time_domain);
    upper_bounds.reserve(2 * kControlDim * config_.control_time_domain);
    for (int row = 0; row < 2 * kControlDim * config_.control_time_domain; ++row) {
      lower_bounds.push_back(static_cast<c_float>(lower(row)));
      upper_bounds.push_back(static_cast<c_float>(upper(row)));
    }

    OSQPWorkspace* workspace = nullptr;
    std::unique_ptr<OSQPData, decltype(&std::free)> data(
        reinterpret_cast<OSQPData*>(std::calloc(1, sizeof(OSQPData))), &std::free);
    std::unique_ptr<OSQPSettings, decltype(&std::free)> settings(
        reinterpret_cast<OSQPSettings*>(std::calloc(1, sizeof(OSQPSettings))), &std::free);
    if (!data || !settings) {
      result.status = "allocation_failed";
      return result;
    }

    osqp_set_default_settings(settings.get());
    settings->verbose = false;
    settings->warm_start = true;

    data->n = static_cast<c_int>(kControlDim * config_.control_time_domain);
    data->m = static_cast<c_int>(2 * kControlDim * config_.control_time_domain);
    data->P = csc_matrix(data->n, data->n, static_cast<c_int>(p_data.size()), p_data.data(),
                         p_indices.data(), p_indptr.data());
    data->q = q_data.data();
    data->A = csc_matrix(data->m, data->n, static_cast<c_int>(a_data.size()), a_data.data(),
                         a_indices.data(), a_indptr.data());
    data->l = lower_bounds.data();
    data->u = upper_bounds.data();

    const c_int setup_status = osqp_setup(&workspace, data.get(), settings.get());
    if (setup_status != 0 || workspace == nullptr) {
      c_free(data->A);
      c_free(data->P);
      result.status = "setup_failed";
      return result;
    }

    osqp_solve(workspace);
    const c_int status = workspace->info ? workspace->info->status_val : -1;
    if (workspace->info && workspace->info->status) {
      result.status = workspace->info->status;
    }

    if ((status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE) &&
        workspace->solution != nullptr && workspace->solution->x != nullptr) {
      result.solved = true;
      result.control =
          Eigen::Vector2d(workspace->solution->x[0] + previous_control_[0],
                          workspace->solution->x[1] + previous_control_[1]);
      result.control[0] = clamp(result.control[0],
                                config_.min_linear_velocity,
                                linear_upper);
      result.control[1] = clamp(result.control[1],
                                -config_.max_angular_velocity,
                                config_.max_angular_velocity);
      previous_control_ = result.control;
    }

    osqp_cleanup(workspace);
    c_free(data->A);
    c_free(data->P);
    return result;
  }

private:
  MpcConfig config_;
  Eigen::Matrix3d q_matrix_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix2d r_matrix_ = Eigen::Matrix2d::Identity();
  Eigen::Vector2d previous_control_ = Eigen::Vector2d::Zero();
};

class MpcControllerNode
{
public:
  MpcControllerNode()
    : nh_(),
      pnh_("~")
  {
    loadParameters();
    controller_.configure(mpc_config_);

    path_sub_ = nh_.subscribe(plan_topic_, 1, &MpcControllerNode::pathCallback, this);
    state_sub_ = nh_.subscribe(state_topic_, 10, &MpcControllerNode::ekfCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MpcControllerNode::odomCallback, this);
    fallback_odom_sub_ = nh_.subscribe(fallback_odom_topic_, 10, &MpcControllerNode::fallbackOdomCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(tracking_status_topic_, 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &MpcControllerNode::controlTimer, this);

    publishStatus("idle");
    ROS_INFO("mpc_controller_node ready: plan='%s', state='%s', cmd='%s'",
             plan_topic_.c_str(), state_topic_.c_str(), cmd_vel_topic_.c_str());
  }

private:
  void loadParameters()
  {
    pnh_.param("state_topic", state_topic_, std::string("/ackermann_car/ekf_state"));
    pnh_.param("odom_topic", odom_topic_, std::string("/ackermann_car/odom"));
    pnh_.param("fallback_odom_topic", fallback_odom_topic_, std::string("/odom"));
    pnh_.param("plan_topic", plan_topic_, std::string("/plan_path"));
    pnh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/ackermann_car/cmd_vel"));
    pnh_.param("tracking_status_topic", tracking_status_topic_, std::string("/tracking_status"));
    pnh_.param("control_rate", control_rate_, 20.0);
    pnh_.param("wheelbase", wheelbase_, 0.60);
    pnh_.param("target_speed", target_speed_, 0.80);
    pnh_.param("min_linear_velocity", mpc_config_.min_linear_velocity, 0.0);
    pnh_.param("max_linear_velocity", mpc_config_.max_linear_velocity, 1.20);
    pnh_.param("max_angular_velocity", mpc_config_.max_angular_velocity, 1.4);
    pnh_.param("max_steering_angle", max_steering_angle_, 0.60);
    pnh_.param("goal_tolerance", goal_tolerance_, 0.35);
    pnh_.param("state_timeout", state_timeout_, 0.75);
    pnh_.param("off_path_stop_distance", off_path_stop_distance_, 1.80);
    pnh_.param("slowdown_distance", slowdown_distance_, 1.5);
    pnh_.param("lookahead_time", lookahead_time_, 1.0);
    pnh_.param("min_lookahead_dist", min_lookahead_dist_, 0.50);
    pnh_.param("max_lookahead_dist", max_lookahead_dist_, 1.40);
    pnh_.param("control_dt", mpc_config_.control_dt, 0.10);
    pnh_.param("predict_time_domain", mpc_config_.predict_time_domain, 10);
    pnh_.param("control_time_domain", mpc_config_.control_time_domain, 4);
    pnh_.param("q_matrix_x", mpc_config_.q_matrix_x, 12.0);
    pnh_.param("q_matrix_y", mpc_config_.q_matrix_y, 12.0);
    pnh_.param("q_matrix_theta", mpc_config_.q_matrix_theta, 3.0);
    pnh_.param("r_matrix_v", mpc_config_.r_matrix_v, 0.8);
    pnh_.param("r_matrix_w", mpc_config_.r_matrix_w, 0.8);
    pnh_.param("delta_linear_velocity_min", mpc_config_.delta_linear_velocity_min, -0.20);
    pnh_.param("delta_linear_velocity_max", mpc_config_.delta_linear_velocity_max, 0.12);
    pnh_.param("delta_angular_velocity_min", mpc_config_.delta_angular_velocity_min, -0.30);
    pnh_.param("delta_angular_velocity_max", mpc_config_.delta_angular_velocity_max, 0.30);

    control_rate_ = std::max(control_rate_, 1.0);
    wheelbase_ = std::max(wheelbase_, 1.0e-3);
    mpc_config_.max_linear_velocity = std::max(mpc_config_.max_linear_velocity, 0.05);
    mpc_config_.min_linear_velocity =
        clamp(mpc_config_.min_linear_velocity, 0.0, mpc_config_.max_linear_velocity);
    target_speed_ = clamp(target_speed_, mpc_config_.min_linear_velocity, mpc_config_.max_linear_velocity);
    mpc_config_.max_angular_velocity = std::max(mpc_config_.max_angular_velocity, 0.05);
    max_steering_angle_ = std::max(max_steering_angle_, 0.05);
    mpc_config_.max_angular_velocity =
        std::min(mpc_config_.max_angular_velocity,
                 mpc_config_.max_linear_velocity * std::tan(max_steering_angle_) / wheelbase_);
    goal_tolerance_ = std::max(goal_tolerance_, 0.01);
    state_timeout_ = std::max(state_timeout_, 0.05);
    off_path_stop_distance_ = std::max(off_path_stop_distance_, 0.05);
    slowdown_distance_ = std::max(slowdown_distance_, goal_tolerance_);
    lookahead_time_ = std::max(lookahead_time_, 0.0);
    min_lookahead_dist_ = std::max(min_lookahead_dist_, 0.05);
    max_lookahead_dist_ = std::max(max_lookahead_dist_, min_lookahead_dist_);
    mpc_config_.control_dt = std::max(mpc_config_.control_dt, 0.01);
    mpc_config_.predict_time_domain = std::max(mpc_config_.predict_time_domain, 1);
    mpc_config_.control_time_domain = std::max(mpc_config_.control_time_domain, 1);
    mpc_config_.q_matrix_x = std::max(mpc_config_.q_matrix_x, 0.0);
    mpc_config_.q_matrix_y = std::max(mpc_config_.q_matrix_y, 0.0);
    mpc_config_.q_matrix_theta = std::max(mpc_config_.q_matrix_theta, 0.0);
    mpc_config_.r_matrix_v = std::max(mpc_config_.r_matrix_v, 1.0e-6);
    mpc_config_.r_matrix_w = std::max(mpc_config_.r_matrix_w, 1.0e-6);
    mpc_config_.delta_linear_velocity_min = std::min(mpc_config_.delta_linear_velocity_min, 0.0);
    mpc_config_.delta_linear_velocity_max = std::max(mpc_config_.delta_linear_velocity_max, 0.0);
    mpc_config_.delta_angular_velocity_min = std::min(mpc_config_.delta_angular_velocity_min, 0.0);
    mpc_config_.delta_angular_velocity_max = std::max(mpc_config_.delta_angular_velocity_max, 0.0);
  }

  void ekfCallback(const ackermann_car_gazebo::RtkImuEkfState::ConstPtr& msg)
  {
    State2d state;
    state.x = msg->x;
    state.y = msg->y;
    state.yaw = msg->yaw;
    state.speed = std::hypot(msg->vx, msg->vy);
    state.yaw_rate = 0.0;
    state.receipt_time = ros::WallTime::now();
    state.valid = isFiniteValue(state.x) && isFiniteValue(state.y) &&
                  isFiniteValue(state.yaw) && isFiniteValue(state.speed);
    ekf_state_ = state;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    primary_odom_state_ = stateFromOdom(*msg);
  }

  void fallbackOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    fallback_odom_state_ = stateFromOdom(*msg);
  }

  State2d stateFromOdom(const nav_msgs::Odometry& msg) const
  {
    State2d state;
    state.x = msg.pose.pose.position.x;
    state.y = msg.pose.pose.position.y;
    state.yaw = quaternionYaw(msg.pose.pose.orientation);
    state.speed = std::hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    state.yaw_rate = msg.twist.twist.angular.z;
    state.receipt_time = ros::WallTime::now();
    state.valid = isFiniteValue(state.x) && isFiniteValue(state.y) &&
                  isFiniteValue(state.yaw) && isFiniteValue(state.speed) &&
                  isFiniteValue(state.yaw_rate);
    return state;
  }

  bool fresh(const State2d& state) const
  {
    return state.valid && (ros::WallTime::now() - state.receipt_time).toSec() <= state_timeout_;
  }

  bool currentState(State2d& state) const
  {
    if (fresh(ekf_state_)) {
      state = ekf_state_;
      if (fresh(primary_odom_state_)) {
        state.speed = primary_odom_state_.speed;
        state.yaw_rate = primary_odom_state_.yaw_rate;
      }
      return true;
    }
    if (fresh(primary_odom_state_)) {
      state = primary_odom_state_;
      return true;
    }
    if (fresh(fallback_odom_state_)) {
      state = fallback_odom_state_;
      return true;
    }
    return false;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    path_.clear();
    if (msg->poses.empty()) {
      resetTracking();
      publishZero();
      publishStatus("idle");
      return;
    }

    path_.reserve(msg->poses.size());
    for (const auto& pose_stamped : msg->poses) {
      const auto& pose = pose_stamped.pose;
      PathPoint point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.yaw = quaternionYaw(pose.orientation);
      if (!path_.empty()) {
        point.s = path_.back().s + distance2d(path_.back().x, path_.back().y, point.x, point.y);
      }
      path_.push_back(point);
    }

    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      path_[i].yaw = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x);
    }
    if (path_.size() >= 2) {
      path_.back().yaw = path_[path_.size() - 2].yaw;
    }

    path_length_ = path_.back().s;
    arrived_ = false;
    progress_s_ = 0.0;
    progress_segment_index_ = 0;
    current_segment_index_ = 0;
    last_linear_ = 0.0;
    last_angular_ = 0.0;
    controller_.reset();
    publishStatus(path_length_ > 1.0e-6 ? "tracking" : "idle");
  }

  void resetTracking()
  {
    path_length_ = 0.0;
    progress_s_ = 0.0;
    progress_segment_index_ = 0;
    current_segment_index_ = 0;
    last_linear_ = 0.0;
    last_angular_ = 0.0;
    arrived_ = false;
    controller_.reset();
  }

  Projection projectToOrderedPath(const State2d& state)
  {
    Projection best;
    if (path_.size() < 2) {
      return best;
    }

    const std::size_t last_segment = path_.size() - 2;
    const std::size_t search_end = std::min(last_segment,
                                           current_segment_index_ +
                                           static_cast<std::size_t>(std::max(3, mpc_config_.predict_time_domain)));
    for (std::size_t i = current_segment_index_; i <= search_end; ++i) {
      const double ax = path_[i].x;
      const double ay = path_[i].y;
      const double bx = path_[i + 1].x;
      const double by = path_[i + 1].y;
      const double vx = bx - ax;
      const double vy = by - ay;
      const double len_sq = vx * vx + vy * vy;
      const double segment_length = path_[i + 1].s - path_[i].s;
      if (len_sq <= 1.0e-12 || segment_length <= 1.0e-9) {
        continue;
      }

      double t = ((state.x - ax) * vx + (state.y - ay) * vy) / len_sq;
      t = clamp(t, 0.0, 1.0);
      const double px = ax + t * vx;
      const double py = ay + t * vy;
      const double distance = distance2d(state.x, state.y, px, py);
      if (distance < best.distance) {
        best.x = px;
        best.y = py;
        best.yaw = std::atan2(vy, vx);
        best.s = path_[i].s + t * segment_length;
        best.distance = distance;
        best.segment_index = i;
      }
    }

    if (!std::isfinite(best.distance)) {
      return projectionAtS(state, progress_s_);
    }

    if (best.s + 1.0e-6 < progress_s_) {
      return projectionAtS(state, progress_s_);
    }

    progress_s_ = best.s;
    current_segment_index_ = best.segment_index;
    progress_segment_index_ = best.segment_index;
    return best;
  }

  Projection projectionAtS(const State2d& state, const double s) const
  {
    const PathPoint point = interpolatePath(s);
    Projection projection;
    projection.x = point.x;
    projection.y = point.y;
    projection.yaw = point.yaw;
    projection.s = point.s;
    projection.distance = distance2d(state.x, state.y, point.x, point.y);
    projection.segment_index = segmentIndexForS(point.s);
    return projection;
  }

  std::size_t segmentIndexForS(const double s) const
  {
    if (path_.size() < 2) {
      return 0;
    }
    std::size_t index = std::min(progress_segment_index_, path_.size() - 2);
    while (index > 0 && path_[index].s > s) {
      --index;
    }
    while (index + 1 < path_.size() && path_[index + 1].s < s) {
      ++index;
    }
    return std::min(index, path_.size() - 2);
  }

  PathPoint interpolatePath(double s) const
  {
    if (path_.empty()) {
      return PathPoint();
    }
    s = clamp(s, 0.0, path_length_);
    if (s <= 0.0) {
      return path_.front();
    }
    if (s >= path_length_) {
      return path_.back();
    }

    for (std::size_t i = progress_segment_index_; i + 1 < path_.size(); ++i) {
      if (path_[i + 1].s < s) {
        continue;
      }
      const double segment_length = path_[i + 1].s - path_[i].s;
      const double t = segment_length <= 1.0e-9 ? 0.0 : (s - path_[i].s) / segment_length;
      PathPoint point;
      point.x = path_[i].x + t * (path_[i + 1].x - path_[i].x);
      point.y = path_[i].y + t * (path_[i + 1].y - path_[i].y);
      point.yaw = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x);
      point.s = s;
      return point;
    }

    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
      if (path_[i + 1].s < s) {
        continue;
      }
      const double segment_length = path_[i + 1].s - path_[i].s;
      const double t = segment_length <= 1.0e-9 ? 0.0 : (s - path_[i].s) / segment_length;
      PathPoint point;
      point.x = path_[i].x + t * (path_[i + 1].x - path_[i].x);
      point.y = path_[i].y + t * (path_[i + 1].y - path_[i].y);
      point.yaw = std::atan2(path_[i + 1].y - path_[i].y, path_[i + 1].x - path_[i].x);
      point.s = s;
      return point;
    }
    return path_.back();
  }

  double curvatureAtS(const double s) const
  {
    if (path_.size() < 3) {
      return 0.0;
    }
    const std::size_t index = segmentIndexForS(s);
    if (index + 1 >= path_.size() - 1) {
      return 0.0;
    }
    const double segment_length = std::max(1.0e-6, path_[index + 1].s - path_[index].s);
    return normalizeAngle(path_[index + 1].yaw - path_[index].yaw) / segment_length;
  }

  void controlTimer(const ros::TimerEvent&)
  {
    if (path_.size() < 2 || path_length_ <= 1.0e-6) {
      publishStatus("idle");
      return;
    }

    State2d state;
    if (!currentState(state)) {
      publishZero();
      publishStatus("error:no_state");
      return;
    }

    const Projection projection = projectToOrderedPath(state);
    const double goal_distance = distance2d(state.x, state.y, path_.back().x, path_.back().y);
    const double remaining = std::max(0.0, path_length_ - projection.s);
    if (goal_distance <= goal_tolerance_ && remaining <= goal_tolerance_) {
      publishZero();
      arrived_ = true;
      publishStatus("arrived");
      return;
    }

    if (arrived_) {
      publishZero();
      publishStatus("arrived");
      return;
    }

    if (projection.distance >= off_path_stop_distance_) {
      publishZero();
      publishStatus("error:off_path");
      return;
    }

    const double lookahead_distance =
        clamp(std::abs(state.speed) * lookahead_time_, min_lookahead_dist_, max_lookahead_dist_);
    const PathPoint desired = interpolatePath(projection.s + lookahead_distance);
    const double curvature = curvatureAtS(projection.s + lookahead_distance);

    double reference_speed = target_speed_;
    if (remaining < slowdown_distance_) {
      reference_speed = std::max(mpc_config_.min_linear_velocity,
                                 target_speed_ * remaining / slowdown_distance_);
    }
    reference_speed = clamp(reference_speed, mpc_config_.min_linear_velocity,
                            mpc_config_.max_linear_velocity);
    const Eigen::Vector2d reference_control(
        reference_speed,
        clamp(reference_speed * curvature,
              -mpc_config_.max_angular_velocity,
              mpc_config_.max_angular_velocity));

    const Eigen::Vector3d current_state(state.x, state.y, state.yaw);
    const Eigen::Vector3d desired_state(desired.x, desired.y, desired.yaw);
    const MpcResult result = controller_.compute(current_state, desired_state,
                                                 reference_control, reference_speed);
    if (!result.solved) {
      publishZero();
      publishStatus("error:optimization");
      ROS_WARN_THROTTLE(1.0, "MPC optimization failed: %s", result.status.c_str());
      return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = result.control[0];
    cmd.angular.z = result.control[1];
    cmd_pub_.publish(cmd);
    last_linear_ = cmd.linear.x;
    last_angular_ = cmd.angular.z;
    publishStatus("tracking");
  }

  void publishZero()
  {
    controller_.reset();
    geometry_msgs::Twist cmd;
    cmd_pub_.publish(cmd);
    last_linear_ = 0.0;
    last_angular_ = 0.0;
  }

  void publishStatus(const std::string& status)
  {
    if (status == last_status_ && (ros::WallTime::now() - last_status_time_).toSec() < 0.5) {
      return;
    }
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
    last_status_ = status;
    last_status_time_ = ros::WallTime::now();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber fallback_odom_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  std::string state_topic_;
  std::string odom_topic_;
  std::string fallback_odom_topic_;
  std::string plan_topic_;
  std::string cmd_vel_topic_;
  std::string tracking_status_topic_;

  double control_rate_ = 20.0;
  double wheelbase_ = 0.60;
  double target_speed_ = 0.80;
  double max_steering_angle_ = 0.60;
  double goal_tolerance_ = 0.35;
  double state_timeout_ = 0.75;
  double off_path_stop_distance_ = 1.80;
  double slowdown_distance_ = 1.5;
  double lookahead_time_ = 1.0;
  double min_lookahead_dist_ = 0.50;
  double max_lookahead_dist_ = 1.40;
  double path_length_ = 0.0;
  double progress_s_ = 0.0;
  std::size_t progress_segment_index_ = 0;
  std::size_t current_segment_index_ = 0;
  double last_linear_ = 0.0;
  double last_angular_ = 0.0;
  bool arrived_ = false;
  std::string last_status_;
  ros::WallTime last_status_time_;

  MpcConfig mpc_config_;
  MpcController controller_;
  State2d ekf_state_;
  State2d primary_odom_state_;
  State2d fallback_odom_state_;
  std::vector<PathPoint> path_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_controller_node");
  MpcControllerNode node;
  ros::spin();
  return 0;
}
