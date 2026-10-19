#ifndef ACKERMANN_CAR_GAZEBO_COVERAGE_PATH_PLANNER_HPP
#define ACKERMANN_CAR_GAZEBO_COVERAGE_PATH_PLANNER_HPP

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace ackermann_car_gazebo
{

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

using LineSegment2 = std::pair<Point2, Point2>;

struct PlanOptions
{
  double scan_width = 1.0;
  double coverage_ratio = 0.8;
  double turn_radius = 0.5;
  double turn_step = 0.1;
  double straight_step = 0.8;
  double start_connection_weight = 1.0;
  std::size_t exact_connection_limit = 16;
  double nonadjacent_penalty = 10.0;
  bool use_robot_start = false;
  bool use_robot_start_heading = false;
  Point2 robot_start;
  double robot_start_heading = 0.0;
};

struct ConnectionState
{
  std::size_t order_index = 0;
  std::size_t region_index = 0;
  int variant_index = -1;
  Point2 entry_point;
  Point2 exit_point;
  double entry_heading = 0.0;
  double exit_heading = 0.0;
  double internal_length = 0.0;
};

struct CoveragePlanResult
{
  bool success = false;
  std::string message;
  std::vector<Point2> path_points;
  std::vector<Point2> scan_path_points;
  std::vector<LineSegment2> lines;
  std::vector<std::vector<Point2> > subregions;
  std::vector<std::vector<bool> > adjacency_matrix;
  std::vector<std::vector<Point2> > turn_paths;
  std::vector<std::vector<Point2> > inter_region_connectors;
  std::vector<Point2> start_connector;
  std::vector<ConnectionState> connection_states;
  std::vector<int> selected_variant_indices;
  std::vector<std::size_t> connection_order;
  std::size_t num_lines = 0;
  std::size_t decomposition_count = 0;
  double line_spacing = 0.0;
  double scan_width = 0.0;
  double coverage_ratio = 0.0;
  double turn_radius = 0.0;
  double turn_step = 0.0;
  double straight_step = 0.0;
  double total_width = 0.0;
  double total_path_cost = 0.0;
  double internal_path_cost = 0.0;
  double smooth_connection_cost = 0.0;
  double start_connection_cost = 0.0;
  double endpoint_preference_cost = 0.0;
  int start_region_index = -1;
  int start_variant_index = -1;
};

class CoveragePathPlanner
{
public:
  CoveragePlanResult plan(const std::vector<Point2>& polygon_points,
                          const PlanOptions& options) const;
};

double pointDistance(const Point2& a, const Point2& b);
double polylineLength(const std::vector<Point2>& points);

}  // namespace ackermann_car_gazebo

#endif  // ACKERMANN_CAR_GAZEBO_COVERAGE_PATH_PLANNER_HPP
