#include <ackermann_car_gazebo/PlanCoveragePath.h>
#include <ackermann_car_gazebo/coverage_path_planner.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <cmath>
#include <string>
#include <vector>

namespace ackermann_car_gazebo
{
namespace
{

geometry_msgs::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

std::vector<Point2> compactPath(const std::vector<Point2>& points)
{
  std::vector<Point2> compacted;
  compacted.reserve(points.size());
  for (const auto& point : points)
  {
    if (compacted.empty() || pointDistance(compacted.back(), point) > 1e-8)
    {
      compacted.push_back(point);
    }
  }
  return compacted;
}

double poseYawAt(const std::vector<Point2>& points, const std::size_t index)
{
  if (points.size() < 2)
  {
    return 0.0;
  }
  if (index + 1 < points.size() && pointDistance(points[index], points[index + 1]) > 1e-8)
  {
    return std::atan2(points[index + 1].y - points[index].y, points[index + 1].x - points[index].x);
  }
  for (std::size_t i = index; i > 0; --i)
  {
    if (pointDistance(points[i - 1], points[index]) > 1e-8)
    {
      return std::atan2(points[index].y - points[i - 1].y, points[index].x - points[i - 1].x);
    }
  }
  return 0.0;
}

nav_msgs::Path toRosPath(const CoveragePlanResult& result,
                         const std::string& frame_id,
                         const ros::Time& stamp)
{
  nav_msgs::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;

  const std::vector<Point2> compacted = compactPath(result.path_points);
  path.poses.reserve(compacted.size());
  for (std::size_t i = 0; i < compacted.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.header.seq = static_cast<std::uint32_t>(i);
    pose.pose.position.x = compacted[i].x;
    pose.pose.position.y = compacted[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = yawToQuaternion(poseYawAt(compacted, i));
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

class CoveragePathPlannerNode
{
public:
  CoveragePathPlannerNode()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("frame_id", frame_id_, "map");
    private_nh_.param("scan_width", default_options_.scan_width, 1.0);
    private_nh_.param("coverage_ratio", default_options_.coverage_ratio, 0.8);
    private_nh_.param("turn_radius", default_options_.turn_radius, 0.5);
    private_nh_.param("turn_step", default_options_.turn_step, 0.1);
    private_nh_.param("straight_step", default_options_.straight_step, 0.8);
    private_nh_.param("start_connection_weight", default_options_.start_connection_weight, 1.0);
    private_nh_.param("nonadjacent_penalty", default_options_.nonadjacent_penalty, 10.0);

    int exact_connection_limit = 16;
    private_nh_.param("exact_connection_limit", exact_connection_limit, 16);
    default_options_.exact_connection_limit = exact_connection_limit > 0 ?
                                                 static_cast<std::size_t>(exact_connection_limit) :
                                                 16U;

    service_ = private_nh_.advertiseService("plan_coverage_path", &CoveragePathPlannerNode::planCallback, this);
    ROS_INFO_STREAM("coverage_path_planner_node ready on " << ros::this_node::getName()
                                                           << "/plan_coverage_path");
  }

private:
  bool planCallback(PlanCoveragePath::Request& request, PlanCoveragePath::Response& response)
  {
    PlanOptions options = default_options_;
    if (request.scan_width > 0.0)
    {
      options.scan_width = request.scan_width;
    }
    if (request.coverage_ratio > 0.0)
    {
      options.coverage_ratio = request.coverage_ratio;
    }
    if (request.turn_radius > 0.0)
    {
      options.turn_radius = request.turn_radius;
    }
    if (request.turn_step > 0.0)
    {
      options.turn_step = request.turn_step;
    }
    if (request.start_connection_weight >= 0.0)
    {
      options.start_connection_weight = request.start_connection_weight;
    }

    options.use_robot_start = request.use_robot_start;
    options.use_robot_start_heading = request.use_robot_start_heading;
    options.robot_start = Point2{request.robot_start.x, request.robot_start.y};
    options.robot_start_heading = request.robot_start.theta;

    std::vector<Point2> polygon;
    polygon.reserve(request.polygon.size());
    for (const auto& point : request.polygon)
    {
      polygon.push_back(Point2{point.x, point.y});
    }

    const CoveragePlanResult result = planner_.plan(polygon, options);
    const std::string frame_id = request.header.frame_id.empty() ? frame_id_ : request.header.frame_id;
    const ros::Time stamp = request.header.stamp.isZero() ? ros::Time::now() : request.header.stamp;

    response.success = result.success;
    response.message = result.message;
    response.path = toRosPath(result, frame_id, stamp);
    response.num_lines = static_cast<std::uint32_t>(result.num_lines);
    response.decomposition_count = static_cast<std::uint32_t>(result.decomposition_count);
    response.line_spacing = result.line_spacing;
    response.total_width = result.total_width;
    response.total_path_cost = result.total_path_cost;
    response.internal_path_cost = result.internal_path_cost;
    response.smooth_connection_cost = result.smooth_connection_cost;
    response.start_connection_cost = result.start_connection_cost;
    response.start_region_index = result.start_region_index;
    response.start_variant_index = result.start_variant_index;
    response.selected_variant_indices = result.selected_variant_indices;
    response.connection_order.clear();
    response.connection_order.reserve(result.connection_order.size());
    for (const std::size_t index : result.connection_order)
    {
      response.connection_order.push_back(static_cast<std::uint32_t>(index));
    }

    if (!result.success)
    {
      ROS_WARN_STREAM("coverage path planning failed: " << result.message);
    }
    return true;
  }

  ros::NodeHandle private_nh_;
  ros::ServiceServer service_;
  CoveragePathPlanner planner_;
  PlanOptions default_options_;
  std::string frame_id_;
};

}  // namespace ackermann_car_gazebo

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coverage_path_planner_node");
  ackermann_car_gazebo::CoveragePathPlannerNode node;
  ros::spin();
  return 0;
}
