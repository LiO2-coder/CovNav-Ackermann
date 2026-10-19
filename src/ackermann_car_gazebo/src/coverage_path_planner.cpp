#include <ackermann_car_gazebo/coverage_path_planner.hpp>

#include <Eigen/Dense>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace ackermann_car_gazebo
{
namespace
{

namespace bg = boost::geometry;

using BgPoint = bg::model::d2::point_xy<double>;
using BgLineString = bg::model::linestring<BgPoint>;
using BgPolygon = bg::model::polygon<BgPoint>;

constexpr double kEpsilon = 1e-9;
constexpr double kTurnEpsilon = 1e-8;

struct Obb2
{
  Point2 center;
  Eigen::Vector2d axes[2];
  double half_extents[2] = {0.0, 0.0};
  double angle = 0.0;
};

struct PathVariant
{
  int variant_index = 0;
  bool reverse_order = false;
  bool flip_first = false;
  std::vector<LineSegment2> directed_segments;
  std::vector<Point2> scan_path_points;
  std::vector<Point2> path_points;
  std::vector<std::vector<Point2> > turn_paths;
  Point2 entry_point;
  Point2 exit_point;
  double entry_heading = 0.0;
  double exit_heading = 0.0;
  double internal_length = 0.0;
};

struct SubregionResult
{
  std::vector<LineSegment2> lines;
  std::vector<Point2> scan_path_points;
  std::vector<Point2> path_points;
  std::vector<std::vector<Point2> > turn_paths;
  std::vector<PathVariant> path_variants;
  std::size_t subregion_index = 0;
  std::vector<Point2> subregion;
  double line_spacing = 0.0;
  double total_width = 0.0;
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  Eigen::Vector2d perp = Eigen::Vector2d::UnitY();
};

struct DecompositionResult
{
  int count = 0;
  std::vector<std::vector<std::size_t> > parts;
};

struct DubinsPath
{
  std::string mode;
  double params[3] = {0.0, 0.0, 0.0};
  double total = 0.0;
};

double sqr(const double value)
{
  return value * value;
}

BgPoint toBg(const Point2& point)
{
  return BgPoint(point.x, point.y);
}

Point2 fromBg(const BgPoint& point)
{
  return Point2{bg::get<0>(point), bg::get<1>(point)};
}

Eigen::Vector2d toEigen(const Point2& point)
{
  return Eigen::Vector2d(point.x, point.y);
}

Point2 fromEigen(const Eigen::Vector2d& point)
{
  return Point2{point.x(), point.y()};
}

double dotPoint(const Point2& point, const Eigen::Vector2d& direction)
{
  return point.x * direction.x() + point.y * direction.y();
}

bool nearlyEqual(const double a, const double b, const double eps = kEpsilon)
{
  return std::abs(a - b) <= eps;
}

bool samePoint(const Point2& a, const Point2& b, const double eps = 1e-8)
{
  return pointDistance(a, b) <= eps;
}

BgPolygon makePolygon(const std::vector<Point2>& points)
{
  BgPolygon polygon;
  polygon.outer().clear();
  for (const auto& point : points)
  {
    polygon.outer().push_back(toBg(point));
  }
  if (!points.empty())
  {
    polygon.outer().push_back(toBg(points.front()));
  }
  bg::correct(polygon);
  return polygon;
}

std::vector<Point2> cleanPolygonPoints(const std::vector<Point2>& points)
{
  if (points.size() < 3)
  {
    throw std::invalid_argument("polygon must contain at least 3 points");
  }

  std::vector<Point2> cleaned;
  cleaned.reserve(points.size());
  for (const auto& point : points)
  {
    if (!cleaned.empty() && pointDistance(cleaned.back(), point) <= kEpsilon)
    {
      continue;
    }
    cleaned.push_back(point);
  }

  if (cleaned.size() > 1 && pointDistance(cleaned.front(), cleaned.back()) <= kEpsilon)
  {
    cleaned.pop_back();
  }
  if (cleaned.size() < 3)
  {
    throw std::invalid_argument("polygon must contain at least 3 distinct points");
  }

  const BgPolygon polygon = makePolygon(cleaned);
  std::string reason;
  if (!bg::is_valid(polygon, reason) || std::abs(bg::area(polygon)) <= 1e-8)
  {
    throw std::invalid_argument("invalid polygon: " + reason);
  }
  return cleaned;
}

double polygonSignedArea(const std::vector<Point2>& points)
{
  double area = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    const Point2& a = points[i];
    const Point2& b = points[(i + 1) % points.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * area;
}

double polygonOrientation(const std::vector<Point2>& points)
{
  return polygonSignedArea(points) >= 0.0 ? 1.0 : -1.0;
}

double cross(const Point2& a, const Point2& b, const Point2& c)
{
  return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

bool isReflexAt(const std::vector<Point2>& points, const std::size_t index, const double orientation)
{
  const Point2& prev = points[(index + points.size() - 1) % points.size()];
  const Point2& current = points[index];
  const Point2& next = points[(index + 1) % points.size()];
  return orientation * cross(prev, current, next) < -1e-8;
}

bool isConvexPolygon(const std::vector<Point2>& points)
{
  if (points.size() <= 3)
  {
    return true;
  }
  const double orientation = polygonOrientation(points);
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    if (orientation * cross(points[(i + points.size() - 1) % points.size()], points[i],
                            points[(i + 1) % points.size()]) < -1e-8)
    {
      return false;
    }
  }
  return true;
}

double orientationValue(const Point2& a, const Point2& b, const Point2& c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(const Point2& a, const Point2& b, const Point2& p)
{
  if (std::abs(orientationValue(a, b, p)) > 1e-8)
  {
    return false;
  }
  return p.x >= std::min(a.x, b.x) - 1e-8 && p.x <= std::max(a.x, b.x) + 1e-8 &&
         p.y >= std::min(a.y, b.y) - 1e-8 && p.y <= std::max(a.y, b.y) + 1e-8;
}

bool segmentsIntersect(const Point2& a, const Point2& b, const Point2& c, const Point2& d)
{
  const double o1 = orientationValue(a, b, c);
  const double o2 = orientationValue(a, b, d);
  const double o3 = orientationValue(c, d, a);
  const double o4 = orientationValue(c, d, b);

  if (((o1 > 1e-8 && o2 < -1e-8) || (o1 < -1e-8 && o2 > 1e-8)) &&
      ((o3 > 1e-8 && o4 < -1e-8) || (o3 < -1e-8 && o4 > 1e-8)))
  {
    return true;
  }

  return onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) || onSegment(c, d, b);
}

bool pointInTriangle(const Point2& point, const Point2& a, const Point2& b, const Point2& c)
{
  const double area1 = orientationValue(point, a, b);
  const double area2 = orientationValue(point, b, c);
  const double area3 = orientationValue(point, c, a);
  const bool has_neg = area1 < -1e-9 || area2 < -1e-9 || area3 < -1e-9;
  const bool has_pos = area1 > 1e-9 || area2 > 1e-9 || area3 > 1e-9;
  return !(has_neg && has_pos);
}

bool isValidDiagonal(const std::vector<Point2>& points, const std::size_t i, const std::size_t j)
{
  if (i == j || std::abs(static_cast<int>(i) - static_cast<int>(j)) <= 1 ||
      (std::min(i, j) == 0 && std::max(i, j) == points.size() - 1))
  {
    return false;
  }

  const BgPolygon polygon = makePolygon(points);
  const Point2 midpoint{0.5 * (points[i].x + points[j].x), 0.5 * (points[i].y + points[j].y)};
  if (!bg::covered_by(toBg(midpoint), polygon))
  {
    return false;
  }

  for (std::size_t edge = 0; edge < points.size(); ++edge)
  {
    const std::size_t next = (edge + 1) % points.size();
    if (edge == i || next == i || edge == j || next == j)
    {
      continue;
    }
    if (segmentsIntersect(points[i], points[j], points[edge], points[next]))
    {
      return false;
    }
  }
  return true;
}

std::vector<std::size_t> indicesBetween(const std::vector<std::size_t>& indices,
                                        const std::size_t start_pos,
                                        const std::size_t end_pos)
{
  std::vector<std::size_t> result;
  if (start_pos <= end_pos)
  {
    result.insert(result.end(), indices.begin() + start_pos, indices.begin() + end_pos + 1);
  }
  else
  {
    result.insert(result.end(), indices.begin() + start_pos, indices.end());
    result.insert(result.end(), indices.begin(), indices.begin() + end_pos + 1);
  }
  return result;
}

std::string indicesKey(const std::vector<std::size_t>& indices)
{
  std::ostringstream stream;
  for (const std::size_t index : indices)
  {
    stream << index << ',';
  }
  return stream.str();
}

std::vector<Point2> pointsForIndices(const std::vector<Point2>& points,
                                     const std::vector<std::size_t>& indices)
{
  std::vector<Point2> result;
  result.reserve(indices.size());
  for (const std::size_t index : indices)
  {
    result.push_back(points[index]);
  }
  return result;
}

DecompositionResult earClipFallbackForIndices(const std::vector<Point2>& points,
                                              const std::vector<std::size_t>& indices)
{
  if (indices.size() <= 3)
  {
    return DecompositionResult{1, {indices}};
  }

  std::vector<std::size_t> polygon_indices = indices;
  std::vector<std::vector<std::size_t> > triangles;
  const double orientation = polygonOrientation(pointsForIndices(points, polygon_indices));
  std::size_t guard = 0;
  while (polygon_indices.size() > 3 && guard++ < indices.size() * indices.size())
  {
    bool ear_found = false;
    const std::size_t count = polygon_indices.size();
    for (std::size_t pos = 0; pos < count; ++pos)
    {
      const std::size_t prev_pos = (pos + count - 1) % count;
      const std::size_t next_pos = (pos + 1) % count;
      const Point2& prev = points[polygon_indices[prev_pos]];
      const Point2& current = points[polygon_indices[pos]];
      const Point2& next = points[polygon_indices[next_pos]];
      if (orientation * orientationValue(prev, current, next) <= 1e-8)
      {
        continue;
      }

      bool contains_point = false;
      for (std::size_t other_pos = 0; other_pos < count; ++other_pos)
      {
        if (other_pos == prev_pos || other_pos == pos || other_pos == next_pos)
        {
          continue;
        }
        if (pointInTriangle(points[polygon_indices[other_pos]], prev, current, next))
        {
          contains_point = true;
          break;
        }
      }
      if (contains_point)
      {
        continue;
      }

      triangles.push_back({polygon_indices[prev_pos], polygon_indices[pos], polygon_indices[next_pos]});
      polygon_indices.erase(polygon_indices.begin() + pos);
      ear_found = true;
      break;
    }

    if (!ear_found)
    {
      return DecompositionResult{1, {indices}};
    }
  }

  if (polygon_indices.size() == 3)
  {
    triangles.push_back(polygon_indices);
  }
  if (triangles.empty())
  {
    triangles.push_back(indices);
  }
  return DecompositionResult{static_cast<int>(triangles.size()), triangles};
}

DecompositionResult solveDecomposition(const std::vector<Point2>& clean_points,
                                       const std::vector<std::size_t>& indices,
                                       std::map<std::string, DecompositionResult>& memo)
{
  const std::string key = indicesKey(indices);
  const auto existing = memo.find(key);
  if (existing != memo.end())
  {
    return existing->second;
  }

  const std::vector<Point2> sub_points = pointsForIndices(clean_points, indices);
  if (indices.size() <= 3 || isConvexPolygon(sub_points))
  {
    DecompositionResult result{1, {indices}};
    memo[key] = result;
    return result;
  }

  const double orientation = polygonOrientation(sub_points);
  std::vector<std::size_t> reflex_positions;
  for (std::size_t pos = 0; pos < indices.size(); ++pos)
  {
    if (isReflexAt(sub_points, pos, orientation))
    {
      reflex_positions.push_back(pos);
    }
  }
  if (reflex_positions.empty())
  {
    DecompositionResult result{1, {indices}};
    memo[key] = result;
    return result;
  }

  int best_count = std::numeric_limits<int>::max();
  std::vector<std::vector<std::size_t> > best_parts;

  for (const std::size_t reflex_pos : reflex_positions)
  {
    for (std::size_t split_pos = 0; split_pos < indices.size(); ++split_pos)
    {
      if (std::abs(static_cast<int>(split_pos) - static_cast<int>(reflex_pos)) <= 1 ||
          (std::min(split_pos, reflex_pos) == 0 && std::max(split_pos, reflex_pos) == indices.size() - 1))
      {
        continue;
      }
      if (!isValidDiagonal(sub_points, reflex_pos, split_pos))
      {
        continue;
      }

      const std::vector<std::size_t> first = indicesBetween(indices, reflex_pos, split_pos);
      const std::vector<std::size_t> second = indicesBetween(indices, split_pos, reflex_pos);
      if (first.size() < 3 || second.size() < 3)
      {
        continue;
      }

      const DecompositionResult left = solveDecomposition(clean_points, first, memo);
      const DecompositionResult right = solveDecomposition(clean_points, second, memo);
      const int total = left.count + right.count;
      if (total < best_count)
      {
        best_count = total;
        best_parts = left.parts;
        best_parts.insert(best_parts.end(), right.parts.begin(), right.parts.end());
      }
    }
  }

  DecompositionResult result;
  if (best_parts.empty())
  {
    result = earClipFallbackForIndices(clean_points, indices);
  }
  else
  {
    result.count = best_count;
    result.parts = best_parts;
  }
  memo[key] = result;
  return result;
}

std::vector<std::vector<Point2> > deduplicateRegions(const std::vector<std::vector<Point2> >& regions)
{
  std::vector<std::vector<Point2> > unique_regions;
  std::set<std::string> seen;
  for (const auto& region : regions)
  {
    if (region.size() < 3 || std::abs(bg::area(makePolygon(region))) <= 1e-8)
    {
      continue;
    }

    std::vector<std::pair<long long, long long> > rounded;
    rounded.reserve(region.size());
    for (const auto& point : region)
    {
      rounded.push_back(std::make_pair(static_cast<long long>(std::llround(point.x * 1e8)),
                                       static_cast<long long>(std::llround(point.y * 1e8))));
    }
    std::sort(rounded.begin(), rounded.end());
    std::ostringstream key;
    for (const auto& item : rounded)
    {
      key << item.first << ':' << item.second << ';';
    }
    if (seen.insert(key.str()).second)
    {
      unique_regions.push_back(region);
    }
  }
  return unique_regions;
}

bool decompositionAreaIsValid(const std::vector<Point2>& points,
                              const std::vector<std::vector<Point2> >& subregions)
{
  const double original_area = std::abs(bg::area(makePolygon(points)));
  double subregion_area = 0.0;
  for (const auto& region : subregions)
  {
    subregion_area += std::abs(bg::area(makePolygon(region)));
  }
  return std::abs(original_area - subregion_area) <= std::max(1e-6, original_area * 1e-6);
}

std::vector<std::vector<Point2> > optimalConvexDecomposition(const std::vector<Point2>& points)
{
  const std::vector<Point2> clean_points = cleanPolygonPoints(points);
  if (isConvexPolygon(clean_points))
  {
    return {clean_points};
  }

  std::vector<std::size_t> vertex_indices(clean_points.size());
  std::iota(vertex_indices.begin(), vertex_indices.end(), 0);
  std::map<std::string, DecompositionResult> memo;
  DecompositionResult result = solveDecomposition(clean_points, vertex_indices, memo);

  std::vector<std::vector<Point2> > subregions;
  subregions.reserve(result.parts.size());
  for (const auto& part : result.parts)
  {
    subregions.push_back(pointsForIndices(clean_points, part));
  }
  subregions = deduplicateRegions(subregions);

  if (subregions.empty() || !decompositionAreaIsValid(clean_points, subregions))
  {
    result = earClipFallbackForIndices(clean_points, vertex_indices);
    subregions.clear();
    for (const auto& part : result.parts)
    {
      subregions.push_back(pointsForIndices(clean_points, part));
    }
    subregions = deduplicateRegions(subregions);
  }

  return subregions.empty() ? std::vector<std::vector<Point2> >{clean_points} : subregions;
}

Eigen::Vector2d normalizeDirection(const Eigen::Vector2d& direction)
{
  const double norm = direction.norm();
  if (norm <= kEpsilon)
  {
    throw std::invalid_argument("direction vector length must be greater than zero");
  }
  Eigen::Vector2d vector = direction / norm;
  if (vector.x() < 0.0 || (std::abs(vector.x()) <= 1e-12 && vector.y() < 0.0))
  {
    vector = -vector;
  }
  return vector;
}

Obb2 computeLocalObb(const std::vector<Point2>& points)
{
  if (points.empty())
  {
    throw std::invalid_argument("cannot compute OBB for empty point set");
  }

  Eigen::Vector2d mean(0.0, 0.0);
  for (const auto& point : points)
  {
    mean += toEigen(point);
  }
  mean /= static_cast<double>(points.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  if (points.size() >= 2)
  {
    for (const auto& point : points)
    {
      const Eigen::Vector2d centered = toEigen(point) - mean;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(points.size() - 1);
  }

  Eigen::Vector2d axis1(1.0, 0.0);
  if (points.size() >= 2)
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() == Eigen::Success)
    {
      axis1 = solver.eigenvectors().col(1);
    }
  }
  axis1 = normalizeDirection(axis1);
  Eigen::Vector2d axis2(-axis1.y(), axis1.x());

  double min1 = std::numeric_limits<double>::infinity();
  double max1 = -std::numeric_limits<double>::infinity();
  double min2 = std::numeric_limits<double>::infinity();
  double max2 = -std::numeric_limits<double>::infinity();
  for (const auto& point : points)
  {
    const Eigen::Vector2d centered = toEigen(point) - mean;
    const double p1 = centered.dot(axis1);
    const double p2 = centered.dot(axis2);
    min1 = std::min(min1, p1);
    max1 = std::max(max1, p1);
    min2 = std::min(min2, p2);
    max2 = std::max(max2, p2);
  }

  const double half_extent1 = 0.5 * (max1 - min1);
  const double half_extent2 = 0.5 * (max2 - min2);
  const Eigen::Vector2d center_adjusted = mean + 0.5 * (min1 + max1) * axis1 + 0.5 * (min2 + max2) * axis2;

  Obb2 obb;
  obb.center = fromEigen(center_adjusted);
  obb.axes[0] = axis1;
  obb.axes[1] = axis2;
  obb.half_extents[0] = half_extent1;
  obb.half_extents[1] = half_extent2;
  obb.angle = std::atan2(axis1.y(), axis1.x());
  return obb;
}

Eigen::Vector2d getObbLongAxis(const Obb2& obb)
{
  const int index = obb.half_extents[0] >= obb.half_extents[1] ? 0 : 1;
  return normalizeDirection(obb.axes[index]);
}

double averageUnsignedAngle(const Eigen::Vector2d& candidate,
                            const std::vector<Eigen::Vector2d>& directions)
{
  double total = 0.0;
  for (const auto& direction : directions)
  {
    const double dot_value = std::max(-1.0, std::min(1.0, std::abs(candidate.dot(direction))));
    total += std::acos(dot_value);
  }
  return directions.empty() ? 0.0 : total / static_cast<double>(directions.size());
}

Eigen::Vector2d recommendDirection(const std::vector<Point2>& subregion,
                                   const std::vector<Point2>& global_polygon,
                                   const std::vector<Eigen::Vector2d>& neighbor_directions)
{
  const Obb2 sub_obb = computeLocalObb(subregion);
  const Obb2 global_obb = computeLocalObb(global_polygon);
  const double long_extent = std::max(sub_obb.half_extents[0], sub_obb.half_extents[1]);
  const double short_extent = std::max(std::min(sub_obb.half_extents[0], sub_obb.half_extents[1]), kEpsilon);
  const double aspect_ratio = long_extent / short_extent;
  if (aspect_ratio > 3.0)
  {
    return getObbLongAxis(sub_obb);
  }

  if (!neighbor_directions.empty())
  {
    const Eigen::Vector2d axis1 = normalizeDirection(sub_obb.axes[0]);
    const Eigen::Vector2d axis2 = normalizeDirection(sub_obb.axes[1]);
    std::vector<Eigen::Vector2d> candidates{axis1, -axis1, axis2, -axis2};
    return *std::min_element(candidates.begin(), candidates.end(),
                             [&](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
                               return averageUnsignedAngle(lhs, neighbor_directions) <
                                      averageUnsignedAngle(rhs, neighbor_directions);
                             });
  }

  return getObbLongAxis(global_obb);
}

double coverageWidth(const std::vector<Point2>& points, const Eigen::Vector2d& direction)
{
  const Eigen::Vector2d perp(-direction.y(), direction.x());
  double min_projection = std::numeric_limits<double>::infinity();
  double max_projection = -std::numeric_limits<double>::infinity();
  for (const auto& point : points)
  {
    const double projection = dotPoint(point, perp);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }
  return max_projection - min_projection;
}

std::vector<std::vector<bool> > buildSubregionAdjacency(const std::vector<std::vector<Point2> >& subregions)
{
  const std::size_t count = subregions.size();
  std::vector<std::vector<bool> > adjacency(count, std::vector<bool>(count, false));
  for (std::size_t i = 0; i < count; ++i)
  {
    adjacency[i][i] = true;
    for (std::size_t j = i + 1; j < count; ++j)
    {
      bool is_neighbor = false;
      for (std::size_t a = 0; a < subregions[i].size() && !is_neighbor; ++a)
      {
        const Point2& a0 = subregions[i][a];
        const Point2& a1 = subregions[i][(a + 1) % subregions[i].size()];
        for (std::size_t b = 0; b < subregions[j].size(); ++b)
        {
          const Point2& b0 = subregions[j][b];
          const Point2& b1 = subregions[j][(b + 1) % subregions[j].size()];
          if ((samePoint(a0, b0) && samePoint(a1, b1)) || (samePoint(a0, b1) && samePoint(a1, b0)))
          {
            if (pointDistance(a0, a1) > 1e-8)
            {
              is_neighbor = true;
              break;
            }
          }
        }
      }
      adjacency[i][j] = is_neighbor;
      adjacency[j][i] = is_neighbor;
    }
  }
  return adjacency;
}

std::vector<double> buildScanOffsets(const double min_proj, const double max_proj, const double line_spacing)
{
  const double total_width = max_proj - min_proj;
  if (total_width <= line_spacing)
  {
    return {0.5 * (min_proj + max_proj)};
  }

  std::vector<double> offsets;
  for (double offset = min_proj + 0.5 * line_spacing; offset < max_proj; offset += line_spacing)
  {
    offsets.push_back(offset);
    if (offsets.size() > 100000)
    {
      throw std::runtime_error("too many scan lines generated; check scan_width and coverage_ratio");
    }
  }
  if (offsets.empty())
  {
    offsets.push_back(0.5 * (min_proj + max_proj));
  }
  return offsets;
}

double lineExtendLength(const std::vector<Point2>& points)
{
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto& point : points)
  {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  return std::max(2.0 * std::hypot(max_x - min_x, max_y - min_y), 1.0);
}

Point2 polygonCentroid(const std::vector<Point2>& points)
{
  const BgPolygon polygon = makePolygon(points);
  BgPoint centroid;
  bg::centroid(polygon, centroid);
  return fromBg(centroid);
}

bool lineStringToSegment(const BgLineString& line,
                         const Eigen::Vector2d& direction,
                         LineSegment2& segment)
{
  if (bg::length(line) <= 1e-9 || line.size() < 2)
  {
    return false;
  }

  Point2 p1 = fromBg(line.front());
  Point2 p2 = fromBg(line.back());
  if (dotPoint(p1, direction) > dotPoint(p2, direction))
  {
    std::swap(p1, p2);
  }

  Eigen::Vector2d start = toEigen(p1);
  Eigen::Vector2d end = toEigen(p2);
  const Eigen::Vector2d vector = end - start;
  const double length = vector.norm();
  const double shrink_length = std::min(length * 1e-9, 1e-7);
  if (length > 2.0 * shrink_length)
  {
    const Eigen::Vector2d unit = vector / length;
    start += shrink_length * unit;
    end -= shrink_length * unit;
  }

  segment = std::make_pair(fromEigen(start), fromEigen(end));
  return true;
}

std::vector<LineSegment2> extractLineSegments(const std::vector<BgLineString>& geometries,
                                              const Eigen::Vector2d& direction)
{
  std::vector<LineSegment2> segments;
  for (const auto& geometry : geometries)
  {
    LineSegment2 segment;
    if (lineStringToSegment(geometry, direction, segment))
    {
      segments.push_back(segment);
    }
  }
  std::sort(segments.begin(), segments.end(), [&](const LineSegment2& lhs, const LineSegment2& rhs) {
    return dotPoint(lhs.first, direction) < dotPoint(rhs.first, direction);
  });
  return segments;
}

double segmentHeading(const Point2& start, const Point2& end)
{
  return std::atan2(end.y - start.y, end.x - start.x);
}

double pathEntryHeading(const std::vector<Point2>& path)
{
  if (path.size() < 2)
  {
    return 0.0;
  }
  const Point2 first = path.front();
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    if (pointDistance(first, path[i]) > kEpsilon)
    {
      return segmentHeading(first, path[i]);
    }
  }
  return 0.0;
}

double pathExitHeading(const std::vector<Point2>& path)
{
  if (path.size() < 2)
  {
    return 0.0;
  }
  const Point2 last = path.back();
  for (std::size_t i = path.size() - 1; i-- > 0;)
  {
    if (pointDistance(path[i], last) > kEpsilon)
    {
      return segmentHeading(path[i], last);
    }
  }
  return 0.0;
}

double mod2pi(const double theta)
{
  const double two_pi = 2.0 * M_PI;
  double result = std::fmod(theta, two_pi);
  if (result < 0.0)
  {
    result += two_pi;
  }
  return result;
}

bool dubinsLsl(const double alpha, const double beta, const double d, double out[3])
{
  const double tmp0 = d + std::sin(alpha) - std::sin(beta);
  const double p_squared = 2.0 + d * d - 2.0 * std::cos(alpha - beta) +
                           2.0 * d * (std::sin(alpha) - std::sin(beta));
  if (p_squared < 0.0)
  {
    return false;
  }
  const double tmp1 = std::atan2(std::cos(beta) - std::cos(alpha), tmp0);
  out[0] = mod2pi(-alpha + tmp1);
  out[1] = std::sqrt(std::max(0.0, p_squared));
  out[2] = mod2pi(beta - tmp1);
  return true;
}

bool dubinsRsr(const double alpha, const double beta, const double d, double out[3])
{
  const double tmp0 = d - std::sin(alpha) + std::sin(beta);
  const double p_squared = 2.0 + d * d - 2.0 * std::cos(alpha - beta) +
                           2.0 * d * (-std::sin(alpha) + std::sin(beta));
  if (p_squared < 0.0)
  {
    return false;
  }
  const double tmp1 = std::atan2(std::cos(alpha) - std::cos(beta), tmp0);
  out[0] = mod2pi(alpha - tmp1);
  out[1] = std::sqrt(std::max(0.0, p_squared));
  out[2] = mod2pi(-beta + tmp1);
  return true;
}

bool dubinsLsr(const double alpha, const double beta, const double d, double out[3])
{
  const double p_squared = -2.0 + d * d + 2.0 * std::cos(alpha - beta) +
                           2.0 * d * (std::sin(alpha) + std::sin(beta));
  if (p_squared < 0.0)
  {
    return false;
  }
  const double p = std::sqrt(std::max(0.0, p_squared));
  const double tmp2 = std::atan2(-std::cos(alpha) - std::cos(beta),
                                 d + std::sin(alpha) + std::sin(beta)) -
                      std::atan2(-2.0, p);
  out[0] = mod2pi(-alpha + tmp2);
  out[1] = p;
  out[2] = mod2pi(-mod2pi(beta) + tmp2);
  return true;
}

bool dubinsRsl(const double alpha, const double beta, const double d, double out[3])
{
  const double p_squared = -2.0 + d * d + 2.0 * std::cos(alpha - beta) -
                           2.0 * d * (std::sin(alpha) + std::sin(beta));
  if (p_squared < 0.0)
  {
    return false;
  }
  const double p = std::sqrt(std::max(0.0, p_squared));
  const double tmp2 = std::atan2(std::cos(alpha) + std::cos(beta),
                                 d - std::sin(alpha) - std::sin(beta)) -
                      std::atan2(2.0, p);
  out[0] = mod2pi(alpha - tmp2);
  out[1] = p;
  out[2] = mod2pi(beta - tmp2);
  return true;
}

bool dubinsRlr(const double alpha, const double beta, const double d, double out[3])
{
  const double tmp0 = (6.0 - d * d + 2.0 * std::cos(alpha - beta) +
                       2.0 * d * (std::sin(alpha) - std::sin(beta))) /
                      8.0;
  if (std::abs(tmp0) > 1.0)
  {
    return false;
  }
  const double p = mod2pi(2.0 * M_PI - std::acos(tmp0));
  const double t = mod2pi(alpha - std::atan2(std::cos(alpha) - std::cos(beta),
                                             d - std::sin(alpha) + std::sin(beta)) +
                          p / 2.0);
  out[0] = t;
  out[1] = p;
  out[2] = mod2pi(alpha - beta - t + p);
  return true;
}

bool dubinsLrl(const double alpha, const double beta, const double d, double out[3])
{
  const double tmp0 = (6.0 - d * d + 2.0 * std::cos(alpha - beta) +
                       2.0 * d * (-std::sin(alpha) + std::sin(beta))) /
                      8.0;
  if (std::abs(tmp0) > 1.0)
  {
    return false;
  }
  const double p = mod2pi(2.0 * M_PI - std::acos(tmp0));
  const double t = mod2pi(-alpha - std::atan2(std::cos(alpha) - std::cos(beta),
                                              d + std::sin(alpha) - std::sin(beta)) +
                          p / 2.0);
  out[0] = t;
  out[1] = p;
  out[2] = mod2pi(beta - alpha - t + p);
  return true;
}

bool shortestDubinsPath(const Point2& start,
                        const double start_heading,
                        const Point2& end,
                        const double end_heading,
                        const double radius,
                        DubinsPath& best_path)
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double distance = std::hypot(dx, dy);
  const double d = distance / radius;
  const double theta = std::atan2(dy, dx);
  const double alpha = mod2pi(start_heading - theta);
  const double beta = mod2pi(end_heading - theta);

  bool found = false;
  const std::vector<std::pair<std::string, bool (*)(double, double, double, double*)> > planners{
      {"LSL", dubinsLsl}, {"RSR", dubinsRsr}, {"LSR", dubinsLsr},
      {"RSL", dubinsRsl}, {"RLR", dubinsRlr}, {"LRL", dubinsLrl}};
  for (const auto& item : planners)
  {
    double params[3] = {0.0, 0.0, 0.0};
    if (!item.second(alpha, beta, d, params))
    {
      continue;
    }
    const double total = params[0] + params[1] + params[2];
    if (!found || total < best_path.total)
    {
      found = true;
      best_path.mode = item.first;
      best_path.params[0] = params[0];
      best_path.params[1] = params[1];
      best_path.params[2] = params[2];
      best_path.total = total;
    }
  }
  return found;
}

std::tuple<double, double, double> propagatePose(const double x,
                                                 const double y,
                                                 const double heading,
                                                 const char segment_type,
                                                 const double length,
                                                 const double radius)
{
  if (segment_type == 'S')
  {
    return std::make_tuple(x + length * std::cos(heading), y + length * std::sin(heading), heading);
  }

  const double turn_sign = segment_type == 'L' ? 1.0 : -1.0;
  const double delta = turn_sign * length / radius;
  const double new_heading = heading + delta;
  return std::make_tuple(x + turn_sign * radius * (std::sin(new_heading) - std::sin(heading)),
                         y - turn_sign * radius * (std::cos(new_heading) - std::cos(heading)), new_heading);
}

std::tuple<double, double, double> dubinsPoseAtDistance(const Point2& start,
                                                       const double start_heading,
                                                       const std::string& mode,
                                                       const double lengths[3],
                                                       const double radius,
                                                       const double distance)
{
  double x = start.x;
  double y = start.y;
  double heading = start_heading;
  double remaining = distance;
  for (std::size_t i = 0; i < 3; ++i)
  {
    const double travel = std::min(remaining, lengths[i]);
    if (travel > 0.0)
    {
      std::tie(x, y, heading) = propagatePose(x, y, heading, mode[i], travel, radius);
    }
    remaining -= travel;
    if (remaining <= 1e-9)
    {
      break;
    }
  }
  return std::make_tuple(x, y, heading);
}

std::vector<Point2> sampleDubinsPath(const Point2& start,
                                     const double start_heading,
                                     const DubinsPath& path,
                                     const double radius,
                                     const double step)
{
  double lengths[3] = {path.params[0] * radius, path.params[1] * radius, path.params[2] * radius};
  const double total_length = lengths[0] + lengths[1] + lengths[2];
  if (total_length <= 1e-9)
  {
    return {start};
  }

  const int sample_count = std::max(1, static_cast<int>(std::ceil(total_length / step)));
  std::vector<Point2> points;
  points.reserve(static_cast<std::size_t>(sample_count) + 2);
  for (int i = 0; i <= sample_count; ++i)
  {
    const double distance = std::min(static_cast<double>(i) * step, total_length);
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    std::tie(x, y, heading) = dubinsPoseAtDistance(start, start_heading, path.mode, lengths, radius, distance);
    (void)heading;
    const Point2 point{x, y};
    if (points.empty() || pointDistance(points.back(), point) > 1e-9)
    {
      points.push_back(point);
    }
  }
  if (points.empty() || pointDistance(points.back(), start) <= 1e-9)
  {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    std::tie(x, y, heading) = dubinsPoseAtDistance(start, start_heading, path.mode, lengths, radius, total_length);
    (void)heading;
    points.push_back(Point2{x, y});
  }
  return points;
}

std::vector<Point2> dubinsPathPoints(const Point2& start,
                                     const double start_heading,
                                     const Point2& end,
                                     const double end_heading,
                                     const double radius,
                                     const double step)
{
  try
  {
    DubinsPath path;
    if (!shortestDubinsPath(start, start_heading, end, end_heading, radius, path))
    {
      return {start, end};
    }
    std::vector<Point2> points = sampleDubinsPath(start, start_heading, path, radius, step);
    if (points.size() >= 2)
    {
      points.front() = start;
      points.back() = end;
    }
    return points;
  }
  catch (const std::exception&)
  {
    return {start, end};
  }
}

std::vector<Point2> smoothConnectorPoints(const Point2& start,
                                          const double start_heading,
                                          const Point2& end,
                                          const double end_heading,
                                          const double turn_radius,
                                          const double turn_step)
{
  std::vector<Point2> turn_points = dubinsPathPoints(start, start_heading, end, end_heading, turn_radius, turn_step);
  if (turn_points.size() >= 2)
  {
    return turn_points;
  }
  return {start, end};
}

std::vector<Point2> sampleLineSegment(const Point2& start, const Point2& end, const double step)
{
  const double length = pointDistance(start, end);
  if (length <= 1e-9)
  {
    return {start};
  }

  const int sample_count = std::max(1, static_cast<int>(std::ceil(length / step)));
  std::vector<Point2> points;
  points.reserve(static_cast<std::size_t>(sample_count) + 1);
  for (int i = 0; i <= sample_count; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
    points.push_back(Point2{start.x + ratio * (end.x - start.x),
                            start.y + ratio * (end.y - start.y)});
  }
  points.front() = start;
  points.back() = end;
  return points;
}

std::vector<LineSegment2> buildDirectedSegmentsWithOptions(const std::vector<LineSegment2>& lines,
                                                           const bool reverse_order,
                                                           const bool flip_first)
{
  std::vector<LineSegment2> ordered_lines = lines;
  if (reverse_order)
  {
    std::reverse(ordered_lines.begin(), ordered_lines.end());
  }

  std::vector<LineSegment2> directed_segments;
  directed_segments.reserve(ordered_lines.size());
  for (std::size_t i = 0; i < ordered_lines.size(); ++i)
  {
    bool forward = (i % 2 == 0);
    if (flip_first)
    {
      forward = !forward;
    }
    if (forward)
    {
      directed_segments.push_back(ordered_lines[i]);
    }
    else
    {
      directed_segments.push_back(std::make_pair(ordered_lines[i].second, ordered_lines[i].first));
    }
  }
  return directed_segments;
}

std::vector<Point2> buildScanPathPoints(const std::vector<LineSegment2>& directed_segments)
{
  std::vector<Point2> points;
  points.reserve(directed_segments.size() * 2);
  for (const auto& segment : directed_segments)
  {
    points.push_back(segment.first);
    points.push_back(segment.second);
  }
  return points;
}

std::pair<std::vector<Point2>, std::vector<std::vector<Point2> > >
buildPathWithDubinsTurns(const std::vector<LineSegment2>& directed_segments,
                         const double turn_radius,
                         const double turn_step,
                         const double straight_step)
{
  std::vector<Point2> path_points;
  std::vector<std::vector<Point2> > turn_paths;
  if (directed_segments.empty())
  {
    return std::make_pair(path_points, turn_paths);
  }

  for (std::size_t i = 0; i < directed_segments.size(); ++i)
  {
    const Point2 start = directed_segments[i].first;
    const Point2 end = directed_segments[i].second;
    const std::vector<Point2> line_points = sampleLineSegment(start, end, straight_step);
    if (path_points.empty())
    {
      path_points.insert(path_points.end(), line_points.begin(), line_points.end());
    }
    else
    {
      const std::size_t start_index =
          pointDistance(path_points.back(), line_points.front()) <= kEpsilon ? 1U : 0U;
      path_points.insert(path_points.end(), line_points.begin() + static_cast<std::ptrdiff_t>(start_index),
                         line_points.end());
    }

    if (i == directed_segments.size() - 1)
    {
      continue;
    }

    const Point2 next_start = directed_segments[i + 1].first;
    const double current_heading = segmentHeading(start, end);
    const double next_heading = segmentHeading(directed_segments[i + 1].first, directed_segments[i + 1].second);
    std::vector<Point2> turn_points =
        dubinsPathPoints(end, current_heading, next_start, next_heading, turn_radius, turn_step);
    if (turn_points.size() < 2)
    {
      turn_points = {end, next_start};
    }
    turn_paths.push_back(turn_points);
    path_points.insert(path_points.end(), turn_points.begin() + 1, turn_points.end());
  }
  return std::make_pair(path_points, turn_paths);
}

std::vector<PathVariant> buildPathVariants(const std::vector<LineSegment2>& lines,
                                           const double turn_radius,
                                           const double turn_step,
                                           const double straight_step)
{
  if (lines.empty())
  {
    return {};
  }

  std::vector<PathVariant> variants;
  const std::vector<std::pair<bool, bool> > options{{false, false}, {false, true}, {true, false}, {true, true}};
  for (std::size_t i = 0; i < options.size(); ++i)
  {
    PathVariant variant;
    variant.variant_index = static_cast<int>(i);
    variant.reverse_order = options[i].first;
    variant.flip_first = options[i].second;
    variant.directed_segments = buildDirectedSegmentsWithOptions(lines, variant.reverse_order, variant.flip_first);
    variant.scan_path_points = buildScanPathPoints(variant.directed_segments);
    std::tie(variant.path_points, variant.turn_paths) =
        buildPathWithDubinsTurns(variant.directed_segments, turn_radius, turn_step, straight_step);
    if (variant.path_points.empty())
    {
      continue;
    }
    variant.entry_point = variant.path_points.front();
    variant.exit_point = variant.path_points.back();
    variant.entry_heading = pathEntryHeading(variant.path_points);
    variant.exit_heading = pathExitHeading(variant.path_points);
    variant.internal_length = polylineLength(variant.path_points);
    variants.push_back(variant);
  }
  return variants;
}

SubregionResult generateBoustrophedonPath(const std::vector<Point2>& polygon_points,
                                          const double scan_width,
                                          const double coverage_ratio,
                                          const double turn_radius,
                                          const double turn_step,
                                          const double straight_step,
                                          const Eigen::Vector2d& direction_vector)
{
  if (scan_width <= 0.0 || coverage_ratio <= 0.0 || turn_radius <= 0.0 || turn_step <= 0.0 ||
      straight_step <= 0.0)
  {
    throw std::invalid_argument(
        "scan_width, coverage_ratio, turn_radius, turn_step and straight_step must be positive");
  }

  const BgPolygon polygon = makePolygon(polygon_points);
  std::string reason;
  if (!bg::is_valid(polygon, reason) || std::abs(bg::area(polygon)) <= 1e-8)
  {
    throw std::invalid_argument("invalid subregion polygon: " + reason);
  }

  const Eigen::Vector2d direction = normalizeDirection(direction_vector);
  const Eigen::Vector2d perp(-direction.y(), direction.x());

  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  for (const auto& point : polygon_points)
  {
    const double projection = dotPoint(point, perp);
    min_proj = std::min(min_proj, projection);
    max_proj = std::max(max_proj, projection);
  }

  const double line_spacing = scan_width * coverage_ratio;
  const std::vector<double> scan_offsets = buildScanOffsets(min_proj, max_proj, line_spacing);
  const double extend_length = lineExtendLength(polygon_points);
  const Point2 centroid = polygonCentroid(polygon_points);
  const Eigen::Vector2d ref_point = toEigen(centroid);

  std::vector<LineSegment2> lines;
  for (const double offset : scan_offsets)
  {
    const Eigen::Vector2d point_on_line = ref_point + (offset - ref_point.dot(perp)) * perp;
    const Eigen::Vector2d line_start = point_on_line - extend_length * direction;
    const Eigen::Vector2d line_end = point_on_line + extend_length * direction;
    BgLineString scan_line;
    scan_line.push_back(toBg(fromEigen(line_start)));
    scan_line.push_back(toBg(fromEigen(line_end)));

    std::vector<BgLineString> clipped;
    bg::intersection(scan_line, polygon, clipped);
    std::vector<LineSegment2> segments = extractLineSegments(clipped, direction);
    lines.insert(lines.end(), segments.begin(), segments.end());
  }

  SubregionResult result;
  result.lines = lines;
  result.path_variants = buildPathVariants(lines, turn_radius, turn_step, straight_step);
  if (!result.path_variants.empty())
  {
    const PathVariant& default_variant = result.path_variants.front();
    result.scan_path_points = default_variant.scan_path_points;
    result.path_points = default_variant.path_points;
    result.turn_paths = default_variant.turn_paths;
  }
  result.line_spacing = line_spacing;
  result.total_width = max_proj - min_proj;
  result.direction = direction;
  result.perp = perp;
  return result;
}

std::vector<Point2> startConnectorPoints(const PlanOptions& options, const PathVariant& variant)
{
  if (!options.use_robot_start)
  {
    return {};
  }
  if (pointDistance(options.robot_start, variant.entry_point) <= kEpsilon)
  {
    return {options.robot_start};
  }
  const double start_heading = options.use_robot_start_heading ?
                                   options.robot_start_heading :
                                   segmentHeading(options.robot_start, variant.entry_point);
  return smoothConnectorPoints(options.robot_start, start_heading, variant.entry_point, variant.entry_heading,
                               options.turn_radius, options.turn_step);
}

double startConnectionCost(const PlanOptions& options, const PathVariant& variant)
{
  if (!options.use_robot_start || options.start_connection_weight <= 0.0)
  {
    return 0.0;
  }
  return polylineLength(startConnectorPoints(options, variant)) * options.start_connection_weight;
}

double variantTransitionCost(const PathVariant& first,
                             const PathVariant& second,
                             const bool adjacent,
                             const PlanOptions& options)
{
  const std::vector<Point2> connector =
      smoothConnectorPoints(first.exit_point, first.exit_heading, second.entry_point, second.entry_heading,
                            options.turn_radius, options.turn_step);
  const double cost = polylineLength(connector);
  return adjacent ? cost : cost * options.nonadjacent_penalty;
}

std::vector<std::vector<PathVariant> > prepareValidVariantSets(
    const std::vector<std::vector<PathVariant> >& all_variant_sets,
    const std::vector<std::vector<bool> >& adjacency_matrix,
    std::vector<std::vector<bool> >& valid_adjacency,
    std::vector<std::size_t>& valid_indices)
{
  valid_indices.clear();
  std::vector<std::vector<PathVariant> > valid_sets;
  for (std::size_t i = 0; i < all_variant_sets.size(); ++i)
  {
    if (!all_variant_sets[i].empty())
    {
      valid_indices.push_back(i);
      valid_sets.push_back(all_variant_sets[i]);
    }
  }

  valid_adjacency.assign(valid_indices.size(), std::vector<bool>(valid_indices.size(), false));
  for (std::size_t i = 0; i < valid_indices.size(); ++i)
  {
    for (std::size_t j = 0; j < valid_indices.size(); ++j)
    {
      valid_adjacency[i][j] = adjacency_matrix[valid_indices[i]][valid_indices[j]];
    }
  }
  return valid_sets;
}

std::vector<std::pair<std::size_t, std::size_t> > connectVariantOrderExact(
    const std::vector<std::vector<PathVariant> >& region_variants,
    const std::vector<std::vector<bool> >& adjacency,
    const std::vector<std::size_t>& original_indices,
    const PlanOptions& options)
{
  const std::size_t count = region_variants.size();
  const std::uint64_t full_mask = (std::uint64_t(1) << count) - 1;
  using State = std::tuple<std::uint64_t, std::size_t, std::size_t>;
  std::map<State, double> dp;
  std::map<State, State> parent;

  for (std::size_t region = 0; region < count; ++region)
  {
    (void)original_indices;
    for (std::size_t variant_index = 0; variant_index < region_variants[region].size(); ++variant_index)
    {
      const PathVariant& variant = region_variants[region][variant_index];
      const State state(std::uint64_t(1) << region, region, variant_index);
      dp[state] = variant.internal_length + startConnectionCost(options, variant);
    }
  }

  for (std::uint64_t mask = 0; mask <= full_mask; ++mask)
  {
    for (std::size_t region = 0; region < count; ++region)
    {
      for (std::size_t variant_index = 0; variant_index < region_variants[region].size(); ++variant_index)
      {
        const State state(mask, region, variant_index);
        const auto current_cost = dp.find(state);
        if (current_cost == dp.end())
        {
          continue;
        }
        const PathVariant& current_variant = region_variants[region][variant_index];
        for (std::size_t next_region = 0; next_region < count; ++next_region)
        {
          if (mask & (std::uint64_t(1) << next_region))
          {
            continue;
          }
          for (std::size_t next_variant_index = 0; next_variant_index < region_variants[next_region].size();
               ++next_variant_index)
          {
            const PathVariant& next_variant = region_variants[next_region][next_variant_index];
            const double transition_cost =
                variantTransitionCost(current_variant, next_variant, adjacency[region][next_region], options);
            const double next_cost = current_cost->second + transition_cost + next_variant.internal_length;
            const State next_state(mask | (std::uint64_t(1) << next_region), next_region, next_variant_index);
            const auto existing = dp.find(next_state);
            if (existing == dp.end() || next_cost < existing->second)
            {
              dp[next_state] = next_cost;
              parent[next_state] = state;
            }
          }
        }
      }
    }
  }

  bool found = false;
  State best_state(0, 0, 0);
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto& item : dp)
  {
    if (std::get<0>(item.first) == full_mask && item.second < best_cost)
    {
      found = true;
      best_cost = item.second;
      best_state = item.first;
    }
  }
  if (!found)
  {
    return {{0, 0}};
  }

  std::vector<std::pair<std::size_t, std::size_t> > order;
  State state = best_state;
  while (true)
  {
    order.push_back(std::make_pair(std::get<1>(state), std::get<2>(state)));
    const auto previous = parent.find(state);
    if (previous == parent.end())
    {
      break;
    }
    state = previous->second;
  }
  std::reverse(order.begin(), order.end());
  return order;
}

std::vector<std::pair<std::size_t, std::size_t> > connectVariantOrderGreedy(
    const std::vector<std::vector<PathVariant> >& region_variants,
    const std::vector<std::vector<bool> >& adjacency,
    const std::vector<std::size_t>& original_indices,
    const PlanOptions& options)
{
  (void)original_indices;
  const std::size_t count = region_variants.size();
  std::vector<std::pair<std::size_t, std::size_t> > best_order;
  double best_cost = std::numeric_limits<double>::infinity();

  for (std::size_t start_region = 0; start_region < count; ++start_region)
  {
    for (std::size_t start_variant_index = 0; start_variant_index < region_variants[start_region].size();
         ++start_variant_index)
    {
      std::set<std::size_t> unvisited;
      for (std::size_t i = 0; i < count; ++i)
      {
        unvisited.insert(i);
      }
      unvisited.erase(start_region);
      std::vector<std::pair<std::size_t, std::size_t> > order{{start_region, start_variant_index}};
      double total = region_variants[start_region][start_variant_index].internal_length +
                     startConnectionCost(options, region_variants[start_region][start_variant_index]);

      while (!unvisited.empty())
      {
        const std::size_t current_region = order.back().first;
        const std::size_t current_variant_index = order.back().second;
        const PathVariant& current_variant = region_variants[current_region][current_variant_index];
        double next_best_cost = std::numeric_limits<double>::infinity();
        std::size_t next_region_best = 0;
        std::size_t next_variant_best = 0;

        for (const std::size_t candidate_region : unvisited)
        {
          for (std::size_t candidate_variant_index = 0;
               candidate_variant_index < region_variants[candidate_region].size(); ++candidate_variant_index)
          {
            const PathVariant& candidate = region_variants[candidate_region][candidate_variant_index];
            const double cost =
                variantTransitionCost(current_variant, candidate, adjacency[current_region][candidate_region],
                                      options) +
                candidate.internal_length;
            if (cost < next_best_cost)
            {
              next_best_cost = cost;
              next_region_best = candidate_region;
              next_variant_best = candidate_variant_index;
            }
          }
        }

        total += next_best_cost;
        order.push_back(std::make_pair(next_region_best, next_variant_best));
        unvisited.erase(next_region_best);
      }

      if (total < best_cost)
      {
        best_cost = total;
        best_order = order;
      }
    }
  }

  return best_order.empty() ? std::vector<std::pair<std::size_t, std::size_t> >{{0, 0}} : best_order;
}

std::vector<std::pair<std::size_t, std::size_t> > connectSubregionVariantStates(
    const std::vector<std::vector<PathVariant> >& region_variants,
    const std::vector<std::vector<bool> >& adjacency,
    const std::vector<std::size_t>& original_indices,
    const PlanOptions& options)
{
  if (region_variants.empty())
  {
    return {};
  }
  if (region_variants.size() <= options.exact_connection_limit && region_variants.size() < 63)
  {
    return connectVariantOrderExact(region_variants, adjacency, original_indices, options);
  }
  return connectVariantOrderGreedy(region_variants, adjacency, original_indices, options);
}

void appendSkippingDuplicate(std::vector<Point2>& target, const std::vector<Point2>& source)
{
  if (source.empty())
  {
    return;
  }
  if (target.empty())
  {
    target.insert(target.end(), source.begin(), source.end());
    return;
  }
  const std::size_t start = pointDistance(target.back(), source.front()) <= kEpsilon ? 1 : 0;
  target.insert(target.end(), source.begin() + static_cast<std::ptrdiff_t>(start), source.end());
}

void assembleVariantConnectedPath(const std::vector<std::vector<PathVariant> >& region_variants,
                                  const std::vector<std::pair<std::size_t, std::size_t> >& states,
                                  const std::vector<std::vector<bool> >& adjacency,
                                  const std::vector<std::size_t>& original_indices,
                                  const PlanOptions& options,
                                  CoveragePlanResult& result)
{
  std::size_t previous_region = 0;
  const PathVariant* previous_variant = nullptr;

  for (std::size_t order_index = 0; order_index < states.size(); ++order_index)
  {
    const std::size_t region = states[order_index].first;
    const std::size_t variant_index = states[order_index].second;
    const PathVariant& variant = region_variants[region][variant_index];
    const std::size_t original_region_index = original_indices[region];

    result.internal_path_cost += variant.internal_length;
    ConnectionState state;
    state.order_index = order_index;
    state.region_index = original_region_index;
    state.variant_index = variant.variant_index;
    state.entry_point = variant.entry_point;
    state.exit_point = variant.exit_point;
    state.entry_heading = variant.entry_heading;
    state.exit_heading = variant.exit_heading;
    state.internal_length = variant.internal_length;
    result.connection_states.push_back(state);
    result.connection_order.push_back(original_region_index);

    if (order_index == 0)
    {
      result.start_connector = startConnectorPoints(options, variant);
      result.start_connection_cost = polylineLength(result.start_connector) * options.start_connection_weight;
      if (!result.start_connector.empty())
      {
        appendSkippingDuplicate(result.path_points, result.start_connector);
        appendSkippingDuplicate(result.path_points, variant.path_points);
      }
      else
      {
        appendSkippingDuplicate(result.path_points, variant.path_points);
      }
      result.start_region_index = static_cast<int>(original_region_index);
      result.start_variant_index = variant.variant_index;
      previous_region = region;
      previous_variant = &variant;
      continue;
    }

    const std::vector<Point2> connector =
        smoothConnectorPoints(previous_variant->exit_point, previous_variant->exit_heading, variant.entry_point,
                              variant.entry_heading, options.turn_radius, options.turn_step);
    const double connector_cost = polylineLength(connector);
    result.smooth_connection_cost +=
        adjacency[previous_region][region] ? connector_cost : connector_cost * options.nonadjacent_penalty;
    result.inter_region_connectors.push_back(connector);
    appendSkippingDuplicate(result.path_points, connector);
    appendSkippingDuplicate(result.path_points, variant.path_points);
    previous_region = region;
    previous_variant = &variant;
  }

  result.total_path_cost =
      result.internal_path_cost + result.smooth_connection_cost + result.start_connection_cost +
      result.endpoint_preference_cost;
}

CoveragePlanResult planImpl(const std::vector<Point2>& polygon_points, const PlanOptions& options)
{
  if (options.scan_width <= 0.0 || options.coverage_ratio <= 0.0 || options.turn_radius <= 0.0 ||
      options.turn_step <= 0.0 || options.straight_step <= 0.0)
  {
    throw std::invalid_argument(
        "scan_width, coverage_ratio, turn_radius, turn_step and straight_step must be positive");
  }
  if (options.start_connection_weight < 0.0)
  {
    throw std::invalid_argument("start_connection_weight must be non-negative");
  }
  if (options.nonadjacent_penalty < 1.0)
  {
    throw std::invalid_argument("nonadjacent_penalty must be greater than or equal to 1");
  }

  const std::vector<Point2> points = cleanPolygonPoints(polygon_points);
  const Obb2 global_obb = computeLocalObb(points);
  const Eigen::Vector2d global_direction = getObbLongAxis(global_obb);

  CoveragePlanResult result;
  result.scan_width = options.scan_width;
  result.coverage_ratio = options.coverage_ratio;
  result.turn_radius = options.turn_radius;
  result.turn_step = options.turn_step;
  result.straight_step = options.straight_step;
  result.line_spacing = options.scan_width * options.coverage_ratio;
  result.total_width = coverageWidth(points, global_direction);

  result.subregions = optimalConvexDecomposition(points);
  result.decomposition_count = result.subregions.size();
  result.adjacency_matrix = buildSubregionAdjacency(result.subregions);

  std::vector<Eigen::Vector2d> directions(result.subregions.size(), Eigen::Vector2d::Zero());
  std::vector<bool> has_direction(result.subregions.size(), false);
  std::vector<std::vector<PathVariant> > all_variant_sets;
  all_variant_sets.reserve(result.subregions.size());

  for (std::size_t index = 0; index < result.subregions.size(); ++index)
  {
    std::vector<Eigen::Vector2d> neighbor_directions;
    for (std::size_t neighbor = 0; neighbor < result.subregions.size(); ++neighbor)
    {
      if (neighbor != index && result.adjacency_matrix[index][neighbor] && has_direction[neighbor])
      {
        neighbor_directions.push_back(normalizeDirection(directions[neighbor]));
      }
    }

    const Eigen::Vector2d direction = recommendDirection(result.subregions[index], points, neighbor_directions);
    directions[index] = direction;
    has_direction[index] = true;

    SubregionResult sub_result = generateBoustrophedonPath(
        result.subregions[index], options.scan_width, options.coverage_ratio, options.turn_radius,
        options.turn_step, options.straight_step, direction);
    sub_result.subregion_index = index;
    sub_result.subregion = result.subregions[index];

    result.lines.insert(result.lines.end(), sub_result.lines.begin(), sub_result.lines.end());
    all_variant_sets.push_back(sub_result.path_variants);
  }

  result.num_lines = result.lines.size();
  result.selected_variant_indices.assign(result.subregions.size(), -1);

  std::vector<std::vector<bool> > valid_adjacency;
  std::vector<std::size_t> valid_indices;
  const std::vector<std::vector<PathVariant> > valid_variant_sets =
      prepareValidVariantSets(all_variant_sets, result.adjacency_matrix, valid_adjacency, valid_indices);

  const std::vector<std::pair<std::size_t, std::size_t> > states =
      connectSubregionVariantStates(valid_variant_sets, valid_adjacency, valid_indices, options);

  for (const auto& state : states)
  {
    const std::size_t original_index = valid_indices[state.first];
    const PathVariant& variant = valid_variant_sets[state.first][state.second];
    result.selected_variant_indices[original_index] = variant.variant_index;
    result.scan_path_points.insert(result.scan_path_points.end(), variant.scan_path_points.begin(),
                                   variant.scan_path_points.end());
    result.turn_paths.insert(result.turn_paths.end(), variant.turn_paths.begin(), variant.turn_paths.end());
  }

  assembleVariantConnectedPath(valid_variant_sets, states, valid_adjacency, valid_indices, options, result);
  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace

double pointDistance(const Point2& a, const Point2& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double polylineLength(const std::vector<Point2>& points)
{
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i)
  {
    length += pointDistance(points[i], points[i + 1]);
  }
  return length;
}

CoveragePlanResult CoveragePathPlanner::plan(const std::vector<Point2>& polygon_points,
                                             const PlanOptions& options) const
{
  try
  {
    return planImpl(polygon_points, options);
  }
  catch (const std::exception& error)
  {
    CoveragePlanResult result;
    result.success = false;
    result.message = error.what();
    return result;
  }
}

}  // namespace ackermann_car_gazebo
